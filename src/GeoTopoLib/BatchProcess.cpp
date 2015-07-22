#include "BatchProcess.h"
#include "myglobals.h"
#include <QGuiApplication>
#include <QApplication>
#include <QGridLayout>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QListWidget>
#include <QMatrix4x4>
#include "EncodeDecodeGeometry.h"

#include "AStarSearch.h"
int AStar::PathSearchNode::k_top = 5;

Structure::ShapeGraph *shapeA, *shapeB;
static QVector<QColor> myrndcolors = rndColors2(100);

QDialog * dialog = nullptr;
BatchProcess * bp = nullptr;

void BatchProcess::init()
{
	bp = this;

	jobUID = 0;

	// Default options
    resultsCount = 3;
	outputPath = "outputPath";
	isDPsearch = false;
	isSwapped = false;
    isSaveReport = false;
    isOutputMatching = false;
	isShowDeformed = false;
	isManyTypesJobs = false;
	isKeepThread = false;
	thumbWidth = 256;
    dpTopK = 20;
    dpTopK_2 = 2;

	// Clean up my self
	connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
	this->connect(this, &BatchProcess::allJobsFinished, [&]{
		if(!this->isKeepThread) this->thread()->quit();
	});

	// Rendering	
	if (isVisualize)
	{
		renderer = QSharedPointer<RenderingWidget>(new RenderingWidget(512, NULL));
		renderer->move(0, 0);
	}

	// Progress
	pd = QSharedPointer<QProgressDialog>(new QProgressDialog("Searching..", "Cancel", 0, 0));
	pd->setValue(0);
	pd->connect(this, SIGNAL(jobFinished(int)), SLOT(setValue(int)));
	pd->connect(this, SIGNAL(allJobsFinished()), SLOT(deleteLater()));
	pd->connect(this, SIGNAL(setLabelText(QString)), SLOT(setLabelText(QString)));

	// Show progress
	if (isVisualize)
	{
		renderer->show();
		pd->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
		pd->show();
	}
}

BatchProcess::BatchProcess(QString filename) : jobfilename(filename), isVisualize(true)
{
	init();

	// Load job's data
	{
		QFile file;
		file.setFileName(filename);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
		QJsonDocument jdoc = QJsonDocument::fromJson(file.readAll());
		auto json = jdoc.object();

		resultsCount = json["resultsCount"].toInt();
		outputPath = json["outputPath"].toString();
		isDPsearch = json["isDPsearch"].toBool();
		isSwapped = json["isSwap"].toBool();
		isSaveReport = json["isSaveReport"].toBool();
		isOutputMatching = json["isOutputMatching"].toBool();
		isShowDeformed = json["isShowDeformed"].toBool();
		thumbWidth = std::max(256, json["thumbWidth"].toInt());
		jobsArray = json["jobs"].toArray();

		if (isDPsearch)
		{
			dpTopK = json.contains("dpTopK") ? json["dpTopK"].toInt() : 20;
            dpTopK_2 = json.contains("dpTopK_2") ? json["dpTopK_2"].toInt() : 2;
		}

        QDir d(""); d.mkpath(outputPath);

		// Clear past results in output folder
        if(json["isCleanOutputFolder"].toBool())
        {
            QDir dir(outputPath);
            for (auto filename : dir.entryList(QDir::Files))
                if (filename.endsWith(".png") || filename.endsWith(".txt") || filename.endsWith(".match"))
                    dir.remove(dir.absolutePath() + "/" + filename);
        }
	}

	// Selective jobs dialog
	if (QGuiApplication::queryKeyboardModifiers().testFlag(Qt::ShiftModifier))
	{
		dialog = new QDialog;
		dialog->setWindowTitle("Select jobs");
		dialog->setModal(true);
		QDialogButtonBox * box = new QDialogButtonBox(Qt::Horizontal);
		auto mainLayout = new QGridLayout;
		auto button = new QPushButton("OK");
		auto buttonSave = new QPushButton("Save selected..");
		button->setDefault(true);
		box->addButton(buttonSave, QDialogButtonBox::NoRole);
		box->addButton(button, QDialogButtonBox::AcceptRole);
		mainLayout->addWidget(box, 1, 0);
		dialog->setLayout(mainLayout);
		dialog->connect(box, SIGNAL(accepted()), SLOT(accept()));
		dialog->setAttribute(Qt::WA_DeleteOnClose);

		// List jobs
		QListWidget * list = new QListWidget;
		for (auto & j : jobsArray){
			auto job = j.toObject(); if (job.isEmpty()) continue;
			QListWidgetItem* item = new QListWidgetItem(job["title"].toString(), list);
			item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
			item->setCheckState(Qt::Unchecked);
			item->setData(Qt::UserRole, job);
		}
		mainLayout->addWidget(list, 0, 0);

		// Actions
		dialog->connect(box, &QDialogButtonBox::accepted, [&]{
			QJsonArray selectedJobs;
			for (int row = 0; row < list->count(); row++){
				QListWidgetItem *item = list->item(row);
				if (item->checkState() == Qt::Checked) selectedJobs << item->data(Qt::UserRole).toJsonObject();
			}
			if (!selectedJobs.isEmpty()) bp->setJobsArray(selectedJobs);
		});
		dialog->connect(box, SIGNAL(accepted()), SLOT(accept()));
		dialog->connect(buttonSave, &QPushButton::clicked, [&]{
			QJsonArray selectedJobs;
			for (int row = 0; row < list->count(); row++){
				QListWidgetItem *item = list->item(row);
				if (item->checkState() == Qt::Checked) selectedJobs << item->data(Qt::UserRole).toJsonObject();
			}
			if (!selectedJobs.isEmpty()) bp->setJobsArray(selectedJobs);
			bp->exportJobFile("currentJobs.json");
			QMessageBox::information(0, "Save", "Jobs saved.");
		});

		// Show
		dialog->exec();
	}	
}

void BatchProcess::run()
{
	QElapsedTimer allTimer; allTimer.start();
	int allTime = 0;

	// Progress
	pd->setMaximum(jobsArray.size());

	for (int idx = 0; idx < jobsArray.size(); idx++ )
	{
		auto & j = jobsArray[idx];

		/// Input Shapes:
		auto job = j.toObject(); if (job.isEmpty()) continue;
		auto source = job["source"].toString();
		auto target = job["target"].toString();
		auto title = job["title"].toString();

		if (isSwapped)
		{
			std::swap(source, target);
			auto splitTitle = title.split("-");
			title = splitTitle.size() > 1 ? splitTitle.back() + "-" + splitTitle.front() : title;
		}

		emit(setLabelText(QString("Corresponding: %1...").arg(title)));

		/// Initial Assignments:
		Energy::Assignments assignments;
		for (auto a : job["assignments"].toArray())
		{
			QStringList la, lb;
			for (auto part : a.toObject()["source"].toArray().toVariantList()) la << part.toString();
			for (auto part : a.toObject()["target"].toArray().toVariantList()) lb << part.toString();
			if (isSwapped) std::swap(la, lb);
			assignments.push_back(qMakePair(la, lb));
		}

		// Useful for debugging jobs
		if (job.contains("isSaveReport")) isSaveReport = job.value("isSaveReport").toBool();

		// Job report
		QVariantMap jobReport;

		// Execute job:
        double execute_cost = executeJob(source, target, job, assignments, jobReport, idx);

        jobReport["execute_cost"].setValue(execute_cost);

		if (isSaveReport)
		{
			auto report_file = QString("%1/%2.job%3.txt").arg(outputPath).arg(title).arg(jobUID);
			QFile file(report_file);
			file.open(QFile::WriteOnly | QFile::Text);
			QTextStream out(&file);
			for (auto key : jobReport.keys())
			{
				out << key << ":" << "\n";

				if (jobReport[key].typeName() == QString("QStringList"))
				{
					for (auto item : jobReport[key].toStringList())
					{
						out << item << "\n";
					}
				}
				else
				{
					out << jobReport[key].toString() << "\n";
				}

				out << "\n======================\n";
			}
		}

        jobReports.push_back(jobReport);

		// Log this result
		if (job["isLogJobs"].toBool())
		{
			QFile file("log.txt");
			if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)){
				QTextStream out(&file);
				out << source << "," << target << "," << jobReport["min_cost"].toDouble() << "\n";
			}
		}
	}

	allTime = allTimer.elapsed();

	emit(jobFinished(jobsArray.size()));
    emit(allJobsFinished());
	emit(reportMessage(QString("Batch process time (%1 s)").arg(double(allTimer.elapsed()) / 1000), 0));
}

double BatchProcess::executeJob(QString sourceFile, QString targetFile, QJsonObject & job, 
	Energy::Assignments & assignments, QVariantMap & jobReport, int jobIdx)
{
	jobUID++;

	// Results:
	QMap <double, Energy::SearchNode> sorted_solutions;
	QVector < QVector <Energy::SearchNode> > solution_vec;

	/// Search solutions:
	Energy::GuidedDeformation egd;

	// Load shapes
    auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(sourceFile));
    auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(targetFile));

	if (shapeA->nodes.isEmpty() || shapeB->nodes.isEmpty()) return 1.0;

	int searchTime = 0;

    if(job["isIgnoreSymmetryGroups"].toBool())
    {
        shapeA->groups.clear();
        shapeB->groups.clear();

        for (auto n : shapeA->nodes) shapeA->setColorFor(n->id, starlab::qRandomColor3());
        for (auto n : shapeB->nodes) shapeB->setColorFor(n->id, starlab::qRandomColor3());
    }

    if (job["isAllowCutsJoins"].toBool())
    {
        //shapeA->performJoins();
        shapeA->performCuts();

        //shapeB->performJoins();
        shapeB->performCuts();
    }

	if (job["isAnisotropy"].toBool())
	{
		QMatrix4x4 mat;
		auto bboxA = shapeA->bbox();
		auto bboxB = shapeB->bbox();
		Vector3 s = bboxA.diagonal().array() / bboxB.diagonal().array();
		mat.scale(s.x(), s.y(), s.z());
		shapeB->transform(mat, true);
	}

    if (job["isFlip"].toBool())
    {
        shapeA->rotate(180,Vector3::UnitZ());
    }

	// Get job title
	auto source = job["source"].toString();
	auto target = job["target"].toString();
	auto title = job["title"].toString();

	if (isSwapped)
	{
        std::swap(source, target);
		auto splitTitle = title.split("-");
		title = splitTitle.size() > 1 ? splitTitle.back() + "-" + splitTitle.front() : title;
	}

	// Set initial correspondence
	QVector<Energy::SearchNode> search_roots;
	Energy::SearchNode path(shapeA, shapeB, QSet<QString>(), assignments);
	path.unassigned = path.unassignedList();
	search_roots << path;

	QElapsedTimer searchTimer; searchTimer.start();

	bool isSearchAstar = true;
	unsigned int numNodesSearched = 0;

	if (isDPsearch)
	{
		auto sourceShape = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeA.data()));
		auto targetShape = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeB.data()));

		egd.K = dpTopK;
		egd.K_2 = dpTopK_2;

		if (job.contains("dpTopK")) egd.K = job["dpTopK"].toInt();
		if (job.contains("dpTopK_2")) egd.K_2 = job["dpTopK_2"].toInt();

		//egd.isApplySYMH = pw->ui->isUseSYMH->isChecked();
		egd.searchDP(sourceShape.data(), targetShape.data(), search_roots);

		if (search_roots.back().shapeA.isNull()) search_roots.pop_back();

		sorted_solutions[EvaluateCorrespondence::evaluate(&search_roots.back())] = search_roots.back();
	}
	else if (isSearchAstar)
	{
		int k_top = 6;

		for (auto & solution : AStar::search(path, 200, k_top, &numNodesSearched))
		{
			egd.origShapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeA));
			egd.origShapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeB));

			solution_vec.push_back(QVector<Energy::SearchNode>());
			for (auto & state : solution) solution_vec.back() << state;
			QVector<Energy::SearchNode*> ptrs;
			for (auto & node : solution_vec.back()) ptrs << &node;
			egd.applySearchPath(ptrs);
			sorted_solutions[solution.back().energy] = solution_vec.back().back();
		}
	}
	else
	{
		// Search for all solutions
		egd.searchAll(shapeA.data(), shapeB.data(), search_roots);

		auto all_solutions = egd.solutions();
		numNodesSearched = all_solutions.size();

		for (auto s : all_solutions)
		{
			double cost = roundDecimal(s->energy, 2);
			sorted_solutions[cost] = *s;
		}
	}

	emit(jobFinished(std::min(jobIdx + 1, jobsArray.size() - 1)));
	QCoreApplication::processEvents();
	searchTime = searchTimer.elapsed();

	double minCostResult = 1.0;

	// Prepare for matching file
	QString match_file;
	if (isManyTypesJobs) match_file = QString("%1/%2.job%3.match").arg(outputPath).arg(title).arg(jobUID);
	else match_file = QString("%1/%2.match").arg(outputPath).arg(title);
	jobReport["match_file"].setValue(match_file);

	/// Draw top solutions:
	QImage img;
	for (int r = 0; r < resultsCount; r++)
	{
		if (r > sorted_solutions.size() - 1) continue; // less solutions than expected

		Energy::SearchNode * selected_path = NULL;
		double cost = sorted_solutions.keys().at(r);

		// Keep track of least cost solution
		minCostResult = std::min(minCostResult, cost);

		if (!isSearchAstar)
		{
			auto entire_path = egd.getEntirePath(&sorted_solutions[cost]);
			egd.applySearchPath(entire_path);
			selected_path = entire_path.back();
		}
		else
		{
			selected_path = &sorted_solutions[cost];
		}

		auto shapeA = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*selected_path->shapeA.data()));
		auto shapeB = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*selected_path->shapeB.data()));

		// Color corresponded nodes
		{
			// Grey out
			for (auto n : shapeA->nodes){
				shapeA->setColorFor(n->id, QColor(100, 100, 100, 255));
				n->vis_property["meshSolid"].setValue(true);
				if (n->type() == Structure::SHEET) ((Structure::Sheet*)n)->surface.quads.clear();
			}
			for (auto n : shapeB->nodes){
				shapeB->setColorFor(n->id, QColor(100, 100, 100, 255));
				n->vis_property["meshSolid"].setValue(true);
				if (n->type() == Structure::SHEET) ((Structure::Sheet*)n)->surface.quads.clear();
			}

			shapeA->property["showNodes"].setValue(false);
			shapeB->property["showNodes"].setValue(false);

			// Matched target nodes
			QSet<QString> matchedTargetNodes;
			for (auto spart : selected_path->mapping.keys())
			{
				auto tpart = selected_path->mapping[spart];
				for (auto group : shapeB->groupsOf(tpart))
					for (auto part : group)
						matchedTargetNodes << part;
			}

			// Assign colors based on target
			int ci = 0;
			for (auto & relation : shapeB->relations)
			{
				QColor color = myrndcolors[ci++];
				for (auto nid : relation.parts)
				{
					if (!matchedTargetNodes.contains(nid)) continue;
					shapeB->setColorFor(nid, color);
					shapeB->getNode(nid)->vis_property["meshSolid"].setValue(true);
				}
			}

			// Color matching source
			for (auto spart : selected_path->mapping.keys())
			{
				auto tpart = selected_path->mapping[spart];
				if (tpart == Structure::null_part) continue;

				auto color = shapeB->getNode(tpart)->vis_property["meshColor"].value<QColor>();

				shapeA->setColorFor(spart, color);
				shapeA->getNode(spart)->vis_property["meshSolid"].setValue(true);
			}
		}

		QImage cur_solution_img;
		if (isVisualize)
		{
			cur_solution_img  = stitchImages(
				renderer->render(shapeA.data()).scaledToWidth(thumbWidth, Qt::TransformationMode::SmoothTransformation),
				renderer->render(shapeB.data()).scaledToWidth(thumbWidth, Qt::TransformationMode::SmoothTransformation));

			renderer->cur_shape = NULL;
		}

		// Show deformed
		if (isShowDeformed)
		{
			auto shapeAcopy = QSharedPointer<Structure::ShapeGraph>(new Structure::ShapeGraph(*shapeA));
			for (auto n : shapeAcopy->nodes)
			{
				if (!n->property.contains("mesh")) continue;

				auto orig_mesh = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data();
				if (!orig_mesh) continue;

				QSharedPointer<SurfaceMeshModel> new_mesh_ptr(orig_mesh->clone());
				new_mesh_ptr->updateBoundingBox();
				new_mesh_ptr->update_face_normals();
				new_mesh_ptr->update_vertex_normals();
				n->property["mesh"].setValue(new_mesh_ptr);
			}

			// Reset shape
			shapeAcopy->setAllControlPoints(shapeAcopy->animation.front());

			// Adjust for splitting cases
			for (auto n : shapeAcopy->nodes){
				if (n->id.contains("@")){
					QString origNode = n->id.split("@").front();
					n->setControlPoints(shapeAcopy->getNode(origNode)->controlPoints());
				}
			}

			// Compute geometry encoding
			ShapeGeometry::encodeGeometry(shapeAcopy.data());

			// Deform abstractions
			shapeAcopy->setAllControlPoints(shapeAcopy->animation.back());

			// Compute deformed underlying surface
			ShapeGeometry::decodeGeometry(shapeAcopy.data());

			if (isVisualize)
			{
				auto deformedImg = renderer->render(shapeAcopy.data()).scaledToWidth(thumbWidth, Qt::TransformationMode::SmoothTransformation);
				renderer->cur_shape = NULL;
				deformedImg = drawText("[Deformed source]", deformedImg, 14, deformedImg.height() - 20);

				cur_solution_img = stitchImages(cur_solution_img, deformedImg);
			}
		}

		// Details of solution if requested
		if (isSaveReport)
		{
			EvaluateCorrespondence::evaluate(selected_path);
			QVariantMap details = selected_path->shapeA->property["costs"].value<QVariantMap>();
			QStringList reportItems;
			for (auto key : details.keys())
			{
				auto detail = details[key].toString();

				// Visualization
				if (key == "zzShapeCost")
				{
					if (isVisualize)
						cur_solution_img = drawText(QString("[%1]").arg(detail), cur_solution_img, 12, cur_solution_img.height() - 20);

					key += QString("(%1)").arg(r);
				}
					
				reportItems += key + " : " + detail;
			}

			reportItems += QString("Total number of parts = %1").arg(egd.origShapeA->nodes.size() + egd.origShapeB->nodes.size());

			// Record found correspondence
			QString mapping;
			for (auto key : selected_path->mapping.keys())
				mapping += QString("(%1,%2) ").arg(key).arg(selected_path->mapping[key]);
			reportItems += mapping;
			jobReport[QString("solution-%1").arg(r)] = reportItems;
		}

		// Visualization
		if (isVisualize)
		{
			cur_solution_img = drawText(QString("s %2: cost = %1").arg(cost).arg(r), cur_solution_img);
			img = stitchImages(img, cur_solution_img, true, 0);
		}

		if (isOutputMatching)
		{
			// Only output best match:
			if (r == 0)
			{
				QFile file(match_file);
				file.open(QFile::WriteOnly | QFile::Text);
				QTextStream out(&file);
				QSet< QString > matchings;

				for (auto key : selected_path->mapping.keys())
				{
					auto sn = selected_path->shapeA->getNode(key);
					auto tn = selected_path->shapeB->getNode(selected_path->mapping[key]);

					// For cutting case + splitting case
					auto realID = [&](Structure::Node * n){
						QString id = n->property.contains("realOriginalID") ? n->property["realOriginalID"].toString() : n->id;
						return id.split("@").front();
					};

					QString sid = realID(sn);
					QString tid = realID(tn);

					QPair<QString, QString> matching;

					// Check for one-to-many case
					bool isSourceOne = !selected_path->shapeA->hasRelation(sid) || selected_path->shapeA->relationOf(sid).parts.size() == 1;
					bool isTargetMany = selected_path->shapeB->hasRelation(tid) && selected_path->shapeB->relationOf(tid).parts.size() > 1;
					if (isSourceOne && isTargetMany)
					{
						for (auto tj : selected_path->shapeB->relationOf(tid).parts)
							matchings << QString("%1|%2").arg(sid).arg(realID(selected_path->shapeB->getNode(tj)));
					}
					else
						matchings << QString("%1|%2").arg(sid).arg(tid);
				}

				for (auto m : matchings)
				{
					auto matching = m.split("|");
					auto sid = matching.front();
					auto tid = matching.back();
					out << QString("%1 %2\n").arg(sid).arg(tid);
				}
			}
		}
	}

	if (isVisualize)
	{
		QString msg = QString("Solution time (%1 s)").arg(double(searchTime) / 1000.0);
		int msgWidth = QFontMetrics(QFont()).width(msg) + 14;
		img = drawText(msg, img, img.width() - msgWidth, 14);
		img = drawText(QString("steps %1").arg(numNodesSearched), img, img.width() - msgWidth, 30);

		auto output_file = QString("%1/%2.job%3.png").arg(outputPath).arg(title).arg(jobUID);
		img.save(output_file);
		std::cout << " Saving image: " << qPrintable(output_file) << "\n";

		jobReport["img_file"].setValue(output_file);
	}

	jobReport["job_uid"].setValue(jobUID);
	jobReport["min_cost"].setValue(minCostResult);
	jobReport["search_time"].setValue(searchTime);

	return minCostResult;
}

void BatchProcess::appendJob(QVariantMap job, QString filename)
{
	QFile file;
	file.setFileName(filename);
	if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) return;
	QJsonDocument jdoc = QJsonDocument::fromJson(file.readAll());
	file.close();

	auto json = jdoc.object();

	// Default values if needed
	if (!json.contains("outputPath"))
	{
        json["outputPath"] = QString("outputPath");
		json["resultsCount"] = 6;
		json["isSaveReport"] = true;
	}

	auto jobs = json["jobs"].toArray();

	auto jj = QJsonObject::fromVariantMap(job);
	jobs.push_front(jj);

	json["jobs"] = jobs;

	QJsonDocument saveDoc(json);
	QFile saveFile(filename);
	if (!saveFile.open(QIODevice::WriteOnly)) return;
	saveFile.write(saveDoc.toJson());
}

void BatchProcess::exportJobFile(QString filename)
{
	if (filename.isEmpty()) filename = jobfilename;

	QJsonObject json;

	json["outputPath"] = outputPath;
	json["resultsCount"] = resultsCount;
	json["isSaveReport"] = isSaveReport;
	json["isOutputMatching"] = isOutputMatching;
	json["jobs"] = jobsArray;

	QJsonDocument saveDoc(json);
	QFile saveFile(filename);
	if (!saveFile.open(QIODevice::WriteOnly)) return;
	saveFile.write(saveDoc.toJson());
}

void BatchProcess::setJobsArray(QJsonArray fromJobsArray)
{
	jobsArray = fromJobsArray;
}

QImage RenderingWidget::render(Structure::ShapeGraph * shape)
{
	QElapsedTimer timer;
	this->cur_shape = shape;
	buffer = QImage();
	this->update();

	// hack blocking to render shapes..
	timer.start();
	while (buffer.isNull()){
		qApp->processEvents();
        if (timer.elapsed() > 10000)
			break;
	}
	
	cur_shape = NULL;
	return buffer;
}

void RenderingWidget::initializeGL()
{
	initializeOpenGLFunctions();
	glClearColor(0.92f, 0.92f, 0.92f, 1);

	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	// Setup lights and material
	GLfloat ambientLightColor[] = { 0.2f, 0.2f, 0.2f, 1 };
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLightColor);

	GLfloat diffuseLightColor[] = { 0.9f, 0.9f, 0.9f, 1 };
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLightColor);

	GLfloat specularLightColor[] = { 0.95f, 0.95f, 0.95f, 1 };
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLightColor);

	float posLight0[] = { 3, 3, 3, 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, posLight0);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	// Specular lighting
	float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	glMaterialfv(GL_FRONT, GL_SPECULAR, specReflection);
	glMateriali(GL_FRONT, GL_SHININESS, 56);
}

void RenderingWidget::paintGL()
{
	if (!cur_shape) return;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_MULTISAMPLE);

    auto bbox = cur_shape->bbox();

	// Setup camera
    qglviewer::Vec cameraPos(-1.5,-1.75,1.0);

	auto s = bbox.diagonal().maxCoeff();
	if (s > 1.0)
	{
		auto delta = cameraPos;
		cameraPos += (delta * s) * 0.3;
	}

	qglviewer::Camera cam;
	cam.setType(qglviewer::Camera::ORTHOGRAPHIC);
	cam.setScreenWidthAndHeight(this->width(), this->height());
	cam.setSceneRadius(20.0f);	
	cam.setSceneCenter(qglviewer::Vec(0, 0, 0.5));
	cam.lookAt(cam.sceneCenter());
	cam.setUpVector(qglviewer::Vec(0, 0, 1));
	cam.setPosition(cameraPos);
	cam.setViewDirection((cam.sceneCenter()-cameraPos).unit());
	cam.loadProjectionMatrix();
	cam.loadModelViewMatrix();

	// Draw shape
	cur_shape->draw();

	// Return QImage of buffer
	auto size = this->size();
	bool alpha_format = true, include_alpha = true;
	QImage img(size, (alpha_format && include_alpha) ? QImage::Format_ARGB32_Premultiplied : QImage::Format_RGB32);
	int w = size.width();
	int h = size.height();
	glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, img.bits());
	convertFromGLImage(img, w, h, alpha_format, include_alpha);

	buffer = img;

	// Reset
	cur_shape = NULL;
}

RenderingWidget::RenderingWidget(int width, QWidget * parent) : cur_shape(NULL), QOpenGLWidget(parent)
{
	setFixedSize(QSize(width, width));
}

// All automatic option
BatchProcess::BatchProcess(QString sourceFilename, QString targetFilename, QVariantMap options) : isVisualize(true)
{
	isVisualize = !options["isQuietMode"].toBool();

	init();

    if (options.contains("k"))
    {
        isDPsearch = true;
        dpTopK_2 = options["k"].value<int>();
    }

	isManyTypesJobs = options["isManyTypesJobs"].toBool();
	isOutputMatching = options["isOutputMatching"].toBool();

	QDir d(""); d.mkpath(outputPath);

	QVariantMap job;
	job["source"].setValue(sourceFilename);
	job["target"].setValue(targetFilename);
	job["title"].setValue(QFileInfo(sourceFilename).baseName() + "-" + QFileInfo(targetFilename).baseName());
    job["isFlip"].setValue(options["isFlip"].toBool());
    job["isAllowCutsJoins"] = options["isAllowCutsJoins"].toBool();
    job["isIgnoreSymmetryGroups"] = options["isIgnoreSymmetryGroups"].toBool();
	job["isLogJobs"] = options["isLogJobs"].toBool();	
	job["isSaveReport"] = options["isSaveReport"].toBool();
	
	jobsArray.push_back(QJsonObject::fromVariantMap(job));
}

BatchProcess::~BatchProcess()
{

}

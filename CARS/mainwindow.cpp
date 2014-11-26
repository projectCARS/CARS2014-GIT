//#include "headers.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "referencedialog.h"
#include "drawsettingsdialog.h"
#include "racedialog.h"

#include "functions.h"
#include "definitions.h"
#include "Calibrator.h"

#include "QThread"

#include <iomanip>
#include <fstream>
#include <sys/stat.h>
#include <QLabel>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Fixates the size of the MainWindow.
    setFixedSize(900,800);

    // If Qt should be used to draw images, instead of OpenCV:
    //QDesktopWidget *desktop = QApplication::desktop();
    //imageLabelProjector->setGeometry(desktop->screenGeometry(1));
    //imageLabelProjector->showFullScreen();
    //imageLabelProjector = new QLabel();
    //imageLabelProjector->show();

    // Set a defualt GUI image.
    ui->imageLabelGui->setFixedSize(880,770);
    QPixmap splashScreen("C:/Users/User/Documents/QtProjects/build-CARS-Desktop_Qt_5_3_MSVC2013_64bit-Release/indata/splashTest.jpg");
    ui->imageLabelGui->setPixmap(splashScreen.scaled(ui->imageLabelGui->width(), ui->imageLabelGui->height(), Qt::KeepAspectRatio));

    // Create threads.
    m_processingThread = new ProcessingThread();
    m_controllerThread = new ControllerThread();
    // Connect signals and slots.
    connect(ui->quitButton, SIGNAL(clicked()), this, SLOT(close()));
    connect(ui->startButton, SIGNAL(clicked()), this, SLOT(startThreads()));
    connect(ui->stopButton, SIGNAL(clicked()), this, SLOT(stopThreads()));
    connect(ui->referenceButton, SIGNAL(clicked()), this, SLOT(openReferenceDialog()));

    m_timer = new QTimer(this);
    // Draw images to screen when the timer signals timeout.
    connect(m_timer, SIGNAL(timeout()), this, SLOT(updateFrame()));

    // OpenCV windows.
    // GUI window.
    // This does not work, since we didn't build OpenCV with OpenGL enabled: cv::namedWindow("GUIWindow", CV_WINDOW_OPENGL);
    cv::namedWindow("GUIWindow", CV_WINDOW_NORMAL);
    cvMoveWindow("GUIWindow", 0, 1024 / 2);
    cv::resizeWindow("GUIWindow", 1280 / 2, 1024 / 2);
    // Projector window.
    cv::namedWindow("ProjectorWindow", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("ProjectorWindow", 1920, 0);
    cvSetWindowProperty("ProjectorWindow", CV_WINDOW_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    HWND win_handle = FindWindow(0, L"ProjectorWindow");
    SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
    ShowWindow(win_handle, SW_SHOW);


    // These variables should initially be zero.
    m_loopCounter = 0;
    m_numCars = 0;

    // Used when calculating FPS.
    m_fpsInterval = 100;
    m_t1 = 0;
}

MainWindow::~MainWindow()
{
    // Close event handles.
    CloseHandle(hControllerThreadEvent1);
    CloseHandle(hDrawThreadEvent);

    // Release resources used by the critical sections.
    DeleteCriticalSection(&csDrawThreadData);
    DeleteCriticalSection(&csControllerThreadData);

    // Destroy OpenCV windows.
    //cv::destroyAllWindows();

    delete ui;
    // delete imageLabelProjector;
    //delete imageLabelGui;
}

void MainWindow::startThreads(void)
{
    m_loopCounter = 0;
    // Load the number of cars from file.
    loadNumCars();
    // Load reference curve from file.
    loadReference();
    // Start threads.
    m_processingThread->start();
    m_controllerThread->start();
    qDebug() << "Threads started.";

    // Load draw settings. Must be done before initFrame().
    loadDrawSettings();
    raceSettings.doRace = m_settings.value("race_settings/do_race").toBool();

    // Initialization.
    initFrame();

    // Enable and disable buttons.
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(true);
    ui->referenceButton->setEnabled(false);
    ui->carSettingsButton->setEnabled(false);
    ui->drawSettingsButton->setEnabled(false);
    ui->calibrateCameraButton->setEnabled(false);
    ui->raceButton->setEnabled(false);

    // Start timer used to draw images.
    // Tell the GUI thread to redraw images as fast as possible by passing the value 0.
    m_timer->start(0);

    // FPS timer.
    m_fpsTime.restart();
}

// Connect this to first race start button
void MainWindow::initializeRace()
{
    m_loopCounter = 0;
    // Load the number of cars from file.
    loadNumCars();
    // Load reference curve from file.
    loadReference();

    // Load draw settings. Must be done before initFrame().
    loadDrawSettings();
    raceSettings.doRace = m_settings.value("race_settings/do_race").toBool();
    int j = 0;
    while(m_settings.contains(QString("race_settings/id%1/race").arg(j)))
    {
            m_settings.beginGroup(QString("race_settings/id%1").arg(j));
            raceSettings.carID.push_back(m_settings.value("race").toInt());
            m_settings.endGroup();
        j++;
    }

    // Initialization.
    EnterCriticalSection(&csDrawThreadData);
    // Save image to file.
    if (!drawThreadData.image.empty())
    {
        std::cout << "Draw thread:		Saving first frame in 'Startup_frame.jpg'" << std::endl;
        imwrite("outdata/Startup_frame.jpg", drawThreadData.image);
    }
    LeaveCriticalSection(&csDrawThreadData);

    // Initialize background images.
    EnterCriticalSection(&csDrawThreadData);
    // Remove old images.
    drawThreadData.background.resize(0);

    // Show animations if enabled.
    if (m_generalDrawSettings & GeneralDrawSettings::Animations)
    {
        for (int i = 1;; i++)
        {
            std::stringstream str;
            str << "indata/background/texture" << std::setw(2) << std::setw(4) << std::setfill('0') << i << ".jpg";
            if (fileExists(str.str()))
            {
                cv::Mat img = cv::imread(str.str(), 1);

                drawThreadData.background.push_back(img);

            }
            else
            {
                std::cout << "Draw thread:		Found " << i - 1 << " background images." << std::endl;
                break;
            }
        }
    }
    else
    {
        // Read static map from file.
        cv::Mat img = cv::imread(m_backgroundPath.toStdString(), CV_8S);
        //cv::Mat img = cv::imread("indata/texture3.jpg", CV_8S);
        drawThreadData.background.push_back(img);
    }
    // Get background image.
    drawThreadData.background[m_loopCounter%drawThreadData.background.size()].copyTo(m_tmpMat);
    LeaveCriticalSection(&csDrawThreadData);

    // Enable and disable buttons.
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(true);
    ui->referenceButton->setEnabled(false);
    ui->carSettingsButton->setEnabled(false);
    ui->drawSettingsButton->setEnabled(false);
    ui->calibrateCameraButton->setEnabled(false);
    ui->raceButton->setEnabled(false);

    // Draw Start boxes
    int numStartBoxes = 0;
    for (int i = 0; i < raceSettings.carID.size(); i++)
    {
        if(raceSettings.carID[i] == 1){
            char output[4];
            sprintf(output,"Car%i", i);
            cv::rectangle(m_tmpMat, cv::Point(800, 30 + 60*numStartBoxes), cv::Point(880, 65 + 60*numStartBoxes), cv::Scalar(255, 255, 255), 1, 8, 0);
            cv::putText(m_tmpMat, output, cv::Point(805, 60 + 60*numStartBoxes), 1, 2, cv::Scalar(255, 255, 50),2,8, false );
            numStartBoxes++;
        }
    }

    cv::imshow("ProjectorWindow", m_tmpMat);
    // Display image from camera.
    EnterCriticalSection(&csDrawThreadData);
    if (!drawThreadData.image.empty())
    {
        cv::imshow("GUIWindow", drawThreadData.image);
    }
    LeaveCriticalSection(&csDrawThreadData);
    // Wait so that OpenCV have time to draw images.
    cv::waitKey(20);
}

// Connect this tosecond race start button
void MainWindow::startRace()
{

    displayCountdown();

    raceSettings.raceStarted = true;
    m_processingThread->start();
    m_controllerThread->start();
    qDebug() << "Threads started.";

    m_timer->start(0);

    // FPS timer.
    m_fpsTime.restart();
}

void MainWindow::displayCountdown(void)
{
    for(int i = 3; i > 1; i --)
    {
        char output[1];
        sprintf(output,"%i", i);
        cv::putText(m_tmpMat, output, cv::Point(500, 500), 1, 8, cv::Scalar(255, 255, 50),2,8, false );

        cv::imshow("ProjectorWindow", m_tmpMat);
        // Display image from camera.
        EnterCriticalSection(&csDrawThreadData);
        if (!drawThreadData.image.empty())
        {
            cv::imshow("GUIWindow", drawThreadData.image);
        }
        LeaveCriticalSection(&csDrawThreadData);
        // Wait so that OpenCV have time to draw images.
        cv::waitKey(20);
        Sleep(1000);
    }
}


void MainWindow::stopThreads(void)
{
    if (m_processingThread->stop())
    {
        SetEvent(hControllerThreadEvent_signalsWritten);
        m_processingThread->wait();
        qDebug() << "Processing thread stopped.";
    }
    else
    {
        qDebug() << "Processing thread have already been stopped.";
    }

    if (m_controllerThread->stop())
    {
        // Prevent thread from waiting on events. Events need to be set in this order.
        SetEvent(hControllerThreadEvent1);
        SetEvent(hControllerThreadEvent_signalsRead);
        m_controllerThread->wait();
        qDebug() << "Controller thread stopped.";
    }
    else
    {
        qDebug() << "Controller thread have already been stopped.";
    }

    // Enable/disable buttons.
    ui->stopButton->setEnabled(false);
    ui->startButton->setEnabled(true);
    ui->referenceButton->setEnabled(true);
    ui->carSettingsButton->setEnabled(true);
    ui->drawSettingsButton->setEnabled(true);
    ui->calibrateCameraButton->setEnabled(true);
    ui->raceButton->setEnabled(true);

    // Stop timer used to draw images.
    m_timer->stop();
}

void MainWindow::loadNumCars(void)
{
    m_numCars = 0;
    // Count the number of cars.
    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        m_numCars++;
    }

    if (m_numCars == 0)
    {
        qDebug() << "m_numCars == 0, in readNumCars(), mainwindow.cpp. No car data could be ";
    }
}

void MainWindow::initFrame(void)
{
    EnterCriticalSection(&csDrawThreadData);
    // Save image to file.
    if (!drawThreadData.image.empty())
    {
        std::cout << "Draw thread:		Saving first frame in 'Startup_frame.jpg'" << std::endl;
        imwrite("outdata/Startup_frame.jpg", drawThreadData.image);
    }
    LeaveCriticalSection(&csDrawThreadData);

    // Initialize background images.
    EnterCriticalSection(&csDrawThreadData);
    // Remove old images.
    drawThreadData.background.resize(0);
    // Show animations if enabled.
    if (m_generalDrawSettings & GeneralDrawSettings::Animations)
    {
        for (int i = 1;; i++)
        {
            std::stringstream str;
            str << "indata/background/texture" << std::setw(2) << std::setw(4) << std::setfill('0') << i << ".jpg";
            if (fileExists(str.str()))
            {
                cv::Mat img = cv::imread(str.str(), 1);

                drawThreadData.background.push_back(img);

            }
            else
            {
                std::cout << "Draw thread:		Found " << i - 1 << " background images." << std::endl;
                break;
            }
        }
    }
    else
    {
        // Read static map from file.
        cv::Mat img = cv::imread(m_backgroundPath.toStdString(), CV_8S);
        //cv::Mat img = cv::imread("indata/texture3.jpg", CV_8S);
        drawThreadData.background.push_back(img);
    }
    LeaveCriticalSection(&csDrawThreadData);

    // Draw reference path on track if enabled.
    if (m_generalDrawSettings & GeneralDrawSettings::Reference)
    {
        EnterCriticalSection(&csDrawThreadData);
        for (int i = 0; i < 2*gRefLen - 2; i = i + 2)
        {
            for (int j = 0; j < drawThreadData.background.size(); j++)
            {
                line(drawThreadData.background[j], cv::Point(gRef[i]*PIXELS_PER_METER, gRef[i + 1]*PIXELS_PER_METER),
                        cv::Point(gRef[i + 2]*PIXELS_PER_METER, gRef[i + 3]*PIXELS_PER_METER), cv::Scalar(0, 0, 0), 2); //(20, 255, 20), 2);
            }
        }
        // Draw a line between the first and last points.
        if (gRefLen > 0)
        {
            for (int j = 0; j < drawThreadData.background.size(); j++)
            {
                line(drawThreadData.background[j], cv::Point(gRef[0]*PIXELS_PER_METER, gRef[1]*PIXELS_PER_METER),
                        cv::Point(gRef[2*gRefLen-4]*PIXELS_PER_METER, gRef[2*gRefLen-3]*PIXELS_PER_METER), cv::Scalar(0, 0, 255), 2);
            }
        }
        LeaveCriticalSection(&csDrawThreadData);
    }

}

void MainWindow::updateFrame(void)
{
    std::vector<float> positionPix(2);
    float angle;
    double timeDiff;
    int numTimers = 0;

    EnterCriticalSection(&csDrawThreadData);
    // Get background image.
    drawThreadData.background[m_loopCounter%drawThreadData.background.size()].copyTo(m_tmpMat);
    raceSettings = drawThreadData.raceData;
    LeaveCriticalSection(&csDrawThreadData);

    EnterCriticalSection(&csDrawThreadData);
    // Get states from struct drawThreadData.
    m_carData = drawThreadData.carData;
    m_carTimerID = drawThreadData.carTimerID;
    LeaveCriticalSection(&csDrawThreadData);

    // Only draw cars if carData is not empty. carData could be empty if the processingthread
    // have not had time to resize it.
    if(raceSettings.raceDone)
    {
        char output[4];
        sprintf(output,"THE WINNER IS CAR %i", raceSettings.winnerID);
        cv::putText(m_tmpMat, output, cv::Point(300, 600), 1, 6, cv::Scalar(255, 255, 50),2,8, false );
    }
    else if (m_carData.size() != 0)
    {
        // Draw Start boxes
        if(!raceSettings.doRace)//raceSettings.doRace && !raceSettings.raceStarted)
        {
            int numStartBoxes = 0;
            for (int i = 0; i < raceSettings.carID.size(); i++)
            {
                if(raceSettings.carID[i] == 1){
                    char output[4];
                    sprintf(output,"Car%i", i);
                    cv::rectangle(m_tmpMat, cv::Point(800, 30 + 60*numStartBoxes), cv::Point(880, 65 + 60*numStartBoxes), cv::Scalar(255, 255, 255), 1, 8, 0);
                    cv::putText(m_tmpMat, output, cv::Point(805, 60 + 60*numStartBoxes), 1, 2, cv::Scalar(255, 255, 50),2,8, false );
                    numStartBoxes++;
                }
            }
        }

        // Draw car on track. The variable j represents car id.
        for (int j = 0; j < m_numCars; j++)
        {
            // Draw Timer for each car to screen
            if(m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Timer)
            {
                if(m_carData[j].lapData.firstLapStarted){
                    char output[4];
                    sprintf(output,"%.1f",m_carData[j].lapData.lapTime);
                    char carID[4];
                    sprintf(carID,"Car %i",j);
                    cv::putText(m_tmpMat, carID, cv::Point((750 -numTimers * 150), 65), 1, 2, cv::Scalar(255,150 ,0), 2, 8, false );
                    cv::putText(m_tmpMat, output, cv::Point((750 -numTimers * 150), 110), 1, 3, cv::Scalar(255, 255, 255), 2, 8, false );

                    if(m_carData[j].lapData.firstLapDone)
                    {
                        char output2[4];
                        sprintf(output2,"%.2f", m_carData[j].lapData.lastLapTime);
                        char output3[4];
                        sprintf(output3,"%.2f", m_carData[j].lapData.bestTime);
                        cv::putText(m_tmpMat, output2, cv::Point((750 -numTimers * 150), 150), 1, 3, cv::Scalar(0, 255, 255),2,8, false );
                        cv::putText(m_tmpMat, output3, cv::Point((750 -numTimers * 150), 190), 1, 3, cv::Scalar(120, 255, 0),2,8, false );
                    }
                }
                numTimers++;
            }

            // Draw Car information to the track
            if ((m_carData[j].active) && (m_carData[j].state.size() != 0))
            {
                // Extract position and angle.
                positionPix[0] = m_carData[j].state[0]*PIXELS_PER_METER;
                positionPix[1] = m_carData[j].state[1]*PIXELS_PER_METER;
                angle = m_carData[j].state[3]*180.0/M_PI;

                cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(positionPix[0], positionPix[1]),
                    cv::Size2f(55, 27
                    ), angle);
                cv::Point2f vertices2f[4];
                cv::Point vertices[4];
                rRect.points(vertices2f);
                for (int i = 0; i < 4; ++i){
                    vertices[i] = vertices2f[i];
                }

                // Draw car.
                // This switch is not neccessary. Instead, all colors can be stored in a vector with elements for each car,
                // thus eliminating the need for the different cases (which reduces the code significantly).
                switch (j)
                {
                case 0:
                    // Draw a rectangle on the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Rectangle)
                    {
                        // Draw a special colour for lost cars.
                        if (m_carData[j].lost)
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(0, 0, 245)); // RED
                        }
                        else
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(204, 255, 51)); // Aqua
                        }
                    }

                    // Draw a circle around the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Circle)
                    {
                        cv::circle(m_tmpMat, cv::Point(positionPix[0], positionPix[1]), sqrt(gCarRadius)*PIXELS_PER_METER, cv::Scalar(255, 255, 0), 1, 8, 0);
                    }

                    break;
                case 1:
                    // Draw a rectangle on the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Rectangle)
                    {
                        // Draw a special colour for lost cars.
                        if (m_carData[j].lost)
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(0, 0, 245)); // RED
                        }
                        else
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(0, 255, 255)); // Yellow
                        }
                    }

                    // Draw a circle around the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Circle)
                    {
                        cv::circle(m_tmpMat, cv::Point(positionPix[0], positionPix[1]), sqrt(gCarRadius)*PIXELS_PER_METER, cv::Scalar(255, 255, 0), 1, 8, 0);
                    }

                    break;
                case 2:
                    // Draw a rectangle on the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Rectangle)
                    {
                        // Draw a special colour for lost cars.
                        if (m_carData[j].lost)
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(0, 0, 245)); // RED
                        }
                        else
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(51, 255, 0)); // Green
                        }
                    }

                    // Draw a circle around the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Circle)
                    {
                        cv::circle(m_tmpMat, cv::Point(positionPix[0], positionPix[1]), sqrt(gCarRadius)*PIXELS_PER_METER, cv::Scalar(51, 255, 0), 1, 8, 0);
                    }

                    break;
                default:

                    // Draw a rectangle on the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Rectangle)
                    {
                        // Draw a special colour for lost cars.
                        if (m_carData[j].lost)
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(0, 0, 245)); // RED
                        }
                        else
                        {
                            cv::fillConvexPoly(m_tmpMat, vertices, 4, cv::Scalar(255, 255, 255)); // White
                        }
                    }

                    // Draw a circle around the car if enabled.
                    if (m_carSpecificDrawSettings[j] & CarSpecificDrawSettings::Circle)
                    {
                        cv::circle(m_tmpMat, cv::Point(positionPix[0], positionPix[1]), sqrt(gCarRadius)*PIXELS_PER_METER, cv::Scalar(255, 255, 0), 1, 8, 0);
                    }
                    break;
                }
            }
        }
    }

    // Display image on projector.
    cv::imshow("ProjectorWindow", m_tmpMat);

    // Display image from camera.
    EnterCriticalSection(&csDrawThreadData);
    if (!drawThreadData.image.empty())
    {
        cv::imshow("GUIWindow", drawThreadData.image);
    }
    LeaveCriticalSection(&csDrawThreadData);

    // Wait so that OpenCV have time to draw images.
    cv::waitKey(20);

    if(raceSettings.raceDone)
    {
        Sleep(1000);
        stopThreads();
    }

    // The images can be displayed using Qt, but that seems to be slower than using OpenCV.
    /*
    // Draw image from camera to GUI.
    EnterCriticalSection(&csDrawThreadData);
    QImage frameGui = matToQImage(drawThreadData.image);
    LeaveCriticalSection(&csDrawThreadData);
    ui->imageLabelGui->setPixmap(QPixmap::fromImage(frameGui).scaled(ui->imageLabelGui->width(), ui->imageLabelGui->height(), Qt::KeepAspectRatio));
    // Draw background with cars to projector.
    QImage frame = matToQImage(m_tmpMat);
    imageLabelProjector->setPixmap(QPixmap::fromImage(frame).scaled(imageLabelProjector->width(), imageLabelProjector->height(), Qt::KeepAspectRatio));
    //imageLabelProjector->update();
    imageLabelProjector->repaint();
    */

    // Calculate FPS.
    if ((m_loopCounter % m_fpsInterval) == 0)
    {
        // Calculate duration of one iteration in ms.
        timeDiff = (double)(m_fpsTime.elapsed()-m_t1) / m_fpsInterval;
        std::cout << "FPS draw: " << 1000.0/timeDiff << std::endl;
        m_t1 = m_fpsTime.elapsed();
    }

    // Increment loop counter.
    m_loopCounter++;
}

QImage MainWindow::matToQImage(const cv::Mat &mat)
{
    if(mat.type() == CV_8UC1)
    {
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
        {
            colorTable.push_back(qRgb(i,i,i));
        }
        const uchar *qImageBuffer = (const uchar*)mat.data;
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img.rgbSwapped();
    }
    else if (mat.type() == CV_8UC3)
    {
        const uchar *qImageBuffer = (const uchar*)mat.data;
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
        qDebug() << "Error: cv::Mat could not be converted to QImage.";
        return QImage();
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // Quit dialog.
    int ret = QMessageBox::question(this,tr("Bilbana"),tr("Are you sure you want to quit?"),
                                    QMessageBox::Cancel | QMessageBox::Yes,
                                    QMessageBox::Yes);

    switch (ret)
    {
        case (QMessageBox::Yes):
            stopThreads();
            // Close Qt labels without parents.
            //imageLabelProjector->close();
            //imageLabelGui->close();
            cv::destroyAllWindows();
            event->accept();
            break;
        default:
            event->ignore();
    }
}

void MainWindow::openReferenceDialog(void)
{
   ReferenceDialog refDialog;
   refDialog.setModal(true);
   refDialog.exec();
}

void MainWindow::on_carSettingsButton_clicked()
{
    CarSettingsDialog dialog;
    dialog.setModal(true);
    dialog.exec();
}

void MainWindow::on_drawSettingsButton_clicked()
{
    DrawSettingsDialog dialog;
    dialog.setModal(true);
    dialog.exec();
}


void MainWindow::on_raceButton_clicked()
{
    raceDialog dialog;
    dialog.setModal(true);
    dialog.exec();
}

void MainWindow::loadReference()
{	// Counter that counts the number of points on the reference curve.
    int numPoints = 0;

    // Open file with reference curve.
    std::ifstream file;
    QString filePath = m_settings.value("reference/file_name").toString();
    file.open(filePath.toStdString().c_str(), std::ios::in);
    if (!file)
    {
        std::cout << "Error: Could not open reference file, in loadReference(), mainWindow.cpp" << std::endl;
        return;
    }
    // First row of reference.txt must be the number of reference points. Then each rows must include: x-coordinate, y-coordinate, speed
    file >> gRefLen;
    // Delete old reference vectors.
    gRef.resize(0);
    vRef.resize(0);
    // Allocate memory for vector with reference curves.
    gRef.resize(gRefLen * 2);
    vRef.resize(gRefLen);
    // Get values from reference curve and convert from pixels to meters.
    for (int i = 0; i < gRefLen; i++)
    {
        // x pixel coordinate.
        file >> gRef[i * 2];
        gRef[i * 2] = gRef[i * 2];
        // y pixel coordinate.
        file >> gRef[i * 2 + 1];
        gRef[i * 2 + 1] = gRef[i * 2 + 1];
        // v_i - speed reference at point (x_i,y_i)
        file >> vRef[i];  //speed must be stored in global coordinates in reference.txt

        numPoints++;
    }

    // Check that there were at least gRefLen numbers to read.
    if (numPoints < gRefLen)
    {
        std::cout << "Error: Number of points on reference curve is smaller than gRefLen (reference file is not formatted properly), in loadReference(), mainWindow.cpp" << std::endl;
    }

    // Close file.
    file.close();
}

void MainWindow::loadDrawSettings(void)
{
    m_carSpecificDrawSettings.resize(0);
    // Number of cars read from settings file.
    int numCarsRead = 0;

    // Add default general draw settings if no such settings are found.
    if (!m_settings.contains("draw_settings/general_draw_settings"))
    {
        // General settings.
        m_generalDrawSettings = 0;
    }
    else
    {
        m_generalDrawSettings = m_settings.value("draw_settings/general_draw_settings").toUInt();
    }

    // Add default background image if no such settings are found.
    if (!m_settings.contains("draw_settings/background"))
    {
        // General settings.
        m_backgroundPath = "C:/Users/User/Documents/QtProjects/build-CARS-Desktop_Qt_5_3_MSVC2013_64bit-Release/indata/worldMap.png";
    }
    else
    {
        m_backgroundPath = m_settings.value("draw_settings/background").toString();
    }

    // Read all draw settings from settings file.
    while (m_settings.contains(QString("car/id%1/mode").arg(numCarsRead)))
    {
        if (m_settings.contains(QString("draw_settings/id%1/car_specific_draw_settings").arg(numCarsRead)))
        {
            m_settings.beginGroup(QString("draw_settings/id%1").arg(numCarsRead));
            m_carSpecificDrawSettings.push_back(m_settings.value("car_specific_draw_settings").toUInt());
            m_settings.endGroup();
        }
        else
        {
            m_carSpecificDrawSettings.push_back((unsigned int)0);
        }

        numCarsRead++;
    }
}

void MainWindow::on_calibrateCameraButton_clicked()
{
    // Create calibrator object
    Calibrator calibrator;
    // Aquire points for projection calibration.
    calibrator.updateCorners();
    int* corners = calibrator.getCornerPoints();

    // Load the map to project the photo on.
    cv::Mat mapImage = cv::imread("indata/worldMap.png", 1);

    // Calculate projection matrices.
    calibrator.calculateCameraToWorldMatrix(mapImage, corners);

    // Save camera matrix, distortion coefficients and projection matrix.
    cv::Mat cameraMatrix = calibrator.getCameraMatrix();
    cv::Mat distCoeffs = calibrator.getDistCoeffs();
    cv::Mat cameraToWorldMatrix = calibrator.getCameraToWorldMatrix();

    cv::FileStorage storage("indata/calibrationData.yml", cv::FileStorage::WRITE);
    storage << "cameraMatrix" << cameraMatrix;
    storage << "distCoeffs" << distCoeffs;
    storage << "cameraToWorldMatrix" << cameraToWorldMatrix;
    storage.release();
}


void MainWindow::on_initRace_clicked()
{
    initializeRace();
}

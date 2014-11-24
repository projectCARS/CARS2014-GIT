#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qtheaders.h"

#include "processingthread.h"
#include "controllerthread.h"
#include "cargroupbox.h"
#include "carsettingsdialog.h"
#include "racedialog.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QLabel *imageLabelProjector;
    QLabel *imageLabelGui;

    QSettings m_settings;
    QImage m_image;

    // Threads.
    ProcessingThread *m_processingThread;
    ControllerThread *m_controllerThread;

    // Timer used to draw images.
    QTimer *m_timer;
    // Time variables.
    QTime m_fpsTime;
    int m_fpsInterval;
    int m_t1;

    std::vector<CarData> m_carData;
    std::vector<int> m_carTimerID;
    int m_loopCounter;
    cv::Mat m_tmpMat;
    int m_numCars;

    // Path to background image.
    QString m_backgroundPath;

    // Variables containing draw settings.
    unsigned int m_generalDrawSettings;
    std::vector<unsigned int> m_carSpecificDrawSettings;

    QImage matToQImage(const cv::Mat &mat);
    // Reads draw settings from file.
    void loadDrawSettings(void);

private slots:
    // Read the number of cars from the settings file.
    void loadNumCars(void);
    // Reads reference curve from file.
    void loadReference(void);
    void initFrame(void);
    void updateFrame(void);
    void startThreads(void);
    void stopThreads(void);
    void closeEvent(QCloseEvent *event);
    void openReferenceDialog(void);

    void on_carSettingsButton_clicked();
    void on_drawSettingsButton_clicked();
    void on_calibrateCameraButton_clicked();
    void on_raceButton_clicked();
};

#endif // MAINWINDOW_H

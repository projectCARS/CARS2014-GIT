#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

#include "qtheaders.h"
#include "classes.h"
#include "definitions.h"

class ProcessingThread : public QThread
{
    Q_OBJECT
public:
    explicit ProcessingThread(QObject *parent = 0);

    // Stop the thread from running.
    bool stop();

protected:
    // The thread runs this function after being created.
    void run();

private:
    // Variables that control whether or not the thread should stop running.
    QMutex m_doStopMutex;
    volatile bool m_doStop;

    // Settings file.
    QSettings m_settings;
    // Number of cars read from file.
    int m_numCars;
    // Vector with cars.
    std::vector<Car> m_cars;

    // Variables containing draw settings.
    unsigned int m_generalDrawSettings;
    std::vector<unsigned int> m_carSpecificDrawSettings;

    QImage matToQImage(const cv::Mat &mat);
    // Reads car settings from file and creates a vector of Car objects.
    void loadCarSettings(void);
    // Reads draw settings from file.
    void loadDrawSettings(void);
    /* Draw states to image. This function needs to be called for each estimation
    of the state vector (in order to obtain a smooth line on the image). */
    void drawStatesToImage(const std::vector<CarData> &oldCarData, const std::vector<CarData> &carData, cv::Mat &evaluatedImage, std::vector<Signal> signal);
    // Find cars from identified markers
    std::vector<CarMeasurement> findCars(const std::vector<float> &dots);
    // Calculates distance between two points.
    float calcDist(float x, float y, float x2, float y2);
    int isThisIntInVector(int i, std::vector<int> vector);
};

#endif // PROCESSINGTHREAD_H

#include "doubleplotdialog.h"
#include "ui_doubleplotdialog.h"
#include "Windows.h"

doubleplotdialog::doubleplotdialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::doubleplotdialog)
{
    ui->setupUi(this);
    ui->plotArea->addGraph();
    ui->plotArea->addGraph();

}

doubleplotdialog::~doubleplotdialog()
{
    delete ui;
}
/*float m_numOfInterval;
float m_intervalLength;
int m_length2mid;
std::vector<int> m_intervalStartIndexes;
std::vector<int> m_intervalMidIndexes;
std::vector<float> m_refSpeedShort;
std::vector<float> m_refSpeedShortBest;
std::vector<float> m_times;
std::vector<float> m_timerTimes;
std::vector<float> m_timesBest;
*/

void doubleplotdialog::firstLeftPlot(float numSections, std::vector<int> sectionMidIndexes, std::vector<float> refSpeed)
{
    qDebug("i first left plot");
    refSpeedStart.resize(numSections);
    leftX.resize(numSections);


    for (int i = 0; i<numSections; i++)
    {
        leftX[i] = sectionMidIndexes[i];
        refSpeedStart[i] = refSpeed[i];
    }

    QVector<double> f = refSpeedStart;
    f[numSections - 1] = 8;

    qDebug("calcs done");

    ui->plotArea->graph(0)->setData(leftX,refSpeedStart);
    ui->plotArea->graph(1)->setData(leftX, f);
    ui->plotArea->xAxis->setRange(0,numSections);
    ui->plotArea->yAxis->setRange(0,15);
    QPen pen;
    pen.setColor(QColor(0, 0, 0));
    ui->plotArea->graph(0)->setPen(pen);
    qDebug("before reploit");

    ui->plotArea->replot();
    qDebug("replot done");
    //Sleep(100);
}



void doubleplotdialog::updatePlots(float numSections, std::vector<float> refSpeedBest, std::vector<float> timesLast, std::vector<float> timesBest )
{
    qDebug("gfdslkagnlkjfda");
    //init
    QVector<double> refSpeedB, timesL, timesB;
    QVector<double> x2(numSections);
    for (int i = 0; i < numSections; i++)
    {
        refSpeedB[i] = refSpeedBest[i];
        timesL[i] = timesLast[i];
        timesB[i] = timesBest[i];
        x2[i] = i;
    }


    //left plot
    ui->plotArea->addGraph();
    ui->plotArea->graph(1)->setData(leftX,refSpeedB);
    //ui->plotArea->xAxis->setRange(0,numSections);
    //ui->leftPlot->yAxis->setRange(0,4);
    QPen pen;
    pen.setColor(QColor(50, 50, 50));
    ui->plotArea->graph(1)->setPen(pen);

    ui->plotArea->replot();

/*
    //right plot
    ui->rigthPlot->addGraph();
    ui->rigthPlot->graph(0)->setData(x2,timesB);
    ui->rigthPlot->xAxis->setRange(0,numSections);
    ui->rigthPlot->yAxis->setRange(0,5);
    pen.setColor(QColor(0, 0, 0));
    ui->rigthPlot->graph(0)->setPen(pen);

    ui->rigthPlot->addGraph();
    ui->rigthPlot->graph(1)->setData(x2,timesL);
    ui->rigthPlot->xAxis->setRange(0,numSections);
    ui->rigthPlot->yAxis->setRange(0,5);
    pen.setColor(QColor(100, 100, 100));
    ui->rigthPlot->graph(1)->setPen(pen);

    ui->rigthPlot->replot();
    Sleep(100);*/
}

void doubleplotdialog::on_pushButton_released()
{
    this->close();
}

void doubleplotdialog::closeEvent(QCloseEvent *event)
{
    event->accept();
}

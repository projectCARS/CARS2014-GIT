#include "doubleplotdialog.h"
#include "ui_doubleplotdialog.h"

doubleplotdialog::doubleplotdialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::doubleplotdialog)
{
    ui->setupUi(this);

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
    refSpeedStart(numSections);
    x(numSections);

    for (int i = 0; i<numSections; i++)
    {
        x[i] = sectionMidIndexes[i];
        refSpeedStart[i] = refSpeed[i];
    }

    ui->leftPlot->addGraph();
    ui->leftPlot->graph(0)->setData(x,refSpeedStart);
    ui->leftPlot->xAxis->setRange(0,numSections);
    ui->leftPlot->yAxis->setRange(0,4);
    QPen pen;
    pen.setColor(QColor(0, 0, 0));
    ui->leftPlot->graph(0)->setPen(pen);

    ui->leftPlot->replot();
}



void doubleplotdialog::updatePlots(float numSections, std::vector<float> refSpeedBest, std::vector<float> timesLast, std::vector<float> timesBest )
{
    //init
    QVector<float> refSpeedB, timesL, timesB;
    QVector<int> x2(numSections);
    for (int i = 0; i < numSections; i++)
    {
        refSpeedB[i] = refSpeedBest[i];
        timesL[i] = timesLast[i];
        timesB[i] = timesBest[i];
    }


    //left plot
    ui->leftPlot->addGraph();
    ui->leftPlot->graph(1)->setData(x,refSpeedB);
    //ui->leftPlot->xAxis->setRange(0,numSections);
    //ui->leftPlot->yAxis->setRange(0,4);
    QPen pen;
    pen.setColor(QColor(50, 50, 50));
    ui->leftPlot->graph(1)->setPen(pen);

    ui->leftPlot->replot();


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
}

#include "plotwindow.h"
#include "ui_plotwindow.h"
#include "definitions.h"
#include "functions.h"
#include "classes.h"
#include "headers.h"
#include <fstream>
#include <iomanip>


plotWindow::plotWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::plotWindow)
{
    ui->setupUi(this);
}

plotWindow::~plotWindow()
{
    delete ui;
}

// create graphs for plot, and sets text labels for title, xaxis and yaxis
void plotWindow::init(PlotData *plotData, QString title, QString xlabel, QString ylabel)
{
    ui->plotArea->clearGraphs();
    ui->plotArea->xAxis->setLabel(xlabel);
    ui->plotArea->yAxis->setLabel(ylabel);

    m_plotData = plotData;
    qDebug() << m_plotData->numOfGraphs;

    ui->titleLabel->setText(title);
    for(int i = 0; i < m_plotData->numOfGraphs; i++)
        ui->plotArea->addGraph();
}

// create legend and specifie names.
void plotWindow::setLegend(int graph, QString leg1)
{
    ui->plotArea->legend->setVisible(true);
    ui->plotArea->graph(graph)->setName(leg1);
}

//update a graph in plot with new values and set axisRange,
void plotWindow::updatePlot(int graph, QVector<double> axisRange, QVector<double> xvalues, QVector<double> yvalues )
{
    ui->plotArea->graph(graph)->setData(xvalues,yvalues);
    ui->plotArea->xAxis->setRange(axisRange[0],axisRange[1]);
    ui->plotArea->yAxis->setRange(axisRange[2],axisRange[3]);

    //Different colors for the different graphs
    QPen pen;
    if (graph == 0)
        pen.setColor(QColor(0, 0, 0)); //black
    else if (graph == 1)
        pen.setColor(QColor(255, 0, 0)); //red
    else
        pen.setColor(QColor(0, 120, 249)); //cyan

    ui->plotArea->graph(graph)->setPen(pen);
    ui->plotArea->replot();
}


void plotWindow::on_closeButton_released()
{
    this->close();
}

void plotWindow::closeEvent(QCloseEvent *event)
{
    event->accept();
}

void plotWindow::on_ExportDataButton_released()
{
    int fileNo = 1;
    std::stringstream str;
    for (int i=0 ; i < 100 ; i++) // Ceiling for number of log files here.
    {
        str.clear();
        str.str(std::string());
        str << "outdata/dataFiles/Data" << m_plotData->name << std::setw(3) << std::setfill('0') << fileNo << ".txt";
        if (!fileExists(str.str()))
        {
            break;
        }
        fileNo++;
    }
    std::ofstream dataStream;
    dataStream.open(str.str());

    for (int k = 0; k < m_plotData->numOfGraphs; k++)
    {
        dataStream << "X" << k << " ";
        for (int i = 0; i< m_plotData->X[k].length(); i++)
        {
            dataStream << m_plotData->X[k][i] << " ";

        }
        dataStream << "\n";
        dataStream << "Y" << k << " ";
        for (int i = 0; i< m_plotData->Y[k].length(); i++)
        {
            dataStream << m_plotData->Y[k][i] << " ";

        }
        dataStream << "\n";

    }
    dataStream.close();
    std::cout << "PlotWindow object:		Writing data to " << str.str() << std::endl;
}

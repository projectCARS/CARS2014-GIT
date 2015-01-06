#include "plotwindow.h"
#include "ui_plotwindow.h"

#include "definitions.h"

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
void plotWindow::init(int numGraphs, QString title, QString xlabel, QString ylabel)
{
    ui->plotArea->clearGraphs();
    ui->plotArea->xAxis->setLabel(xlabel);
    ui->plotArea->yAxis->setLabel(ylabel);

    ui->titleLabel->setText(title);
    for(int i = 0; i<numGraphs; i++)
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

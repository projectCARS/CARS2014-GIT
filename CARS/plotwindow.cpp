#include "plotwindow.h"
#include "ui_plotwindow.h"

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

void plotWindow::init(int numGraphs, QString title, QString xlabel, QString ylabel)
{

    ui->plotArea->xAxis->setLabel(xlabel);
    ui->plotArea->yAxis->setLabel(ylabel);

    ui->titleLabel->setText(title);
    for(int i = 0; i<numGraphs; i++)
        ui->plotArea->addGraph();
}

void plotWindow::updatePlot(int graph, QVector<double> axisRange, QVector<double> xvalues, QVector<double> yvalues )
{

    ui->plotArea->graph(graph)->setData(xvalues,yvalues);
    ui->plotArea->xAxis->setRange(axisRange[0],axisRange[1]);
    ui->plotArea->yAxis->setRange(axisRange[2],axisRange[3]);

    QPen pen;
    if (graph = 0)
        pen.setColor(QColor(0, 0, 0)); //black
    else
        pen.setColor(QColor(50, 50, 50)); //red

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

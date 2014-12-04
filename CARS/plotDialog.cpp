#include "plotDialog.h"
#include "ui_plotDialog.h"

plotDialog::plotDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::plotDialog)
{
    ui->setupUi(this);
}

plotDialog::~plotDialog()
{
    delete ui;
}


plotDialog::makePlot(int lengthX)
{
    QVector<double> x(lengthX), y(lengthX);
    for (int i = 0; i<lengthX; i++)
    {
        x[i] = i;
        y[i] = i;

        ui->plot->addGraph();
        ui->plot->graph(0)->setData(x,y);
        ui->plot->xAxis->setRange(0,lengthX+10);
        ui->plot->yAxis->setRange(0,lengthX+5);
    }
}

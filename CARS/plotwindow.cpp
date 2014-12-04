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

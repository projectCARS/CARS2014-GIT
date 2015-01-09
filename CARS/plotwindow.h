#ifndef PLOTWINDOW_H
#define PLOTWINDOW_H

#include <QWidget>
#include "qtheaders.h"
#include "classes.h"
#include "definitions.h"

namespace Ui {
class plotWindow;
}

class plotWindow : public QWidget
{
    Q_OBJECT

public:
    explicit plotWindow(QWidget *parent = 0);
    ~plotWindow();
    void init(PlotData *plotData, QString title, QString xlabel, QString ylabel);
    void setLegend(int graph, QString leg1);

    void updatePlot(int graph, QVector<double> axisRange, QVector<double> xvalues, QVector<double> yvalues );

private slots:
    void on_closeButton_released();

    void on_ExportDataButton_released();

private:
    Ui::plotWindow *ui;

    void closeEvent(QCloseEvent *event);

    PlotData *m_plotData;

    QVector<double> Xvalues;
    QVector<double> Yvalues0;
    QVector<double> Yvalues1;
};

#endif // PLOTWINDOW_H

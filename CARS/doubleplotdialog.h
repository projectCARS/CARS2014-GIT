#ifndef DOUBLEPLOTDIALOG_H
#define DOUBLEPLOTDIALOG_H

#include <QDialog>

namespace Ui {
class doubleplotdialog;
}

class doubleplotdialog : public QDialog
{
    Q_OBJECT

public:
    explicit doubleplotdialog(QWidget *parent = 0);
    ~doubleplotdialog();
    void firstLeftPlot(float numSections, std::vector<int> sectionMidIndexes, std::vector<float> refSpeed);
    void updatePlots(float numSections, std::vector<float> refSpeedBest, std::vector<float> timesLast, std::vector<float> timesBest );

private:
    Ui::doubleplotdialog *ui;

    QVector<double> refSpeedStart;
    QVector<double> leftX;
};

#endif // DOUBLEPLOTDIALOG_H

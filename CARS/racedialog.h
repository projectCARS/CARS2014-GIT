#ifndef RACEDIALOG_H
#define RACEDIALOG_H

#include <QDialog>
#include "qtheaders.h"

#include "processingthread.h"
#include "controllerthread.h"
#include "racegroupbox.h"
#include "carsettingsdialog.h"
#include "racedialog.h"

namespace Ui {
class raceDialog;
}

class raceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit raceDialog(QWidget *parent = 0);
    ~raceDialog();

private slots:
    void on_startRaceButton_released();

    void on_carSettingsButton_clicked();

private:
    Ui::raceDialog *ui;

    void addRaceGroupBox(void);
    std::vector<RaceGroupBox*> m_raceGroupBoxes;
    QSettings m_settings;
    int m_numCars;

    void closeEvent(QCloseEvent *event);

};

#endif // RACEDIALOG_H

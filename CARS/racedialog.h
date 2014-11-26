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

    void on_decreaseLapsButton_clicked();

    void on_increaseLapsButton_clicked();

private:
    Ui::raceDialog *ui;

    void addRaceGroupBox(void);
    std::vector<RaceGroupBox*> m_raceGroupBoxes;
    std::vector<int> m_raceCarID;
    int lapNumber = 5;

    QSettings m_settings;
    int m_numCars = 0;

    void closeEvent(QCloseEvent *event);
    void saveRaceSettings(void);

};


#endif // RACEDIALOG_H

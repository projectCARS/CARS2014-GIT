#ifndef RACEDIALOG_H
#define RACEDIALOG_H

#include <QDialog>
#include "qtheaders.h"

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

private:
    Ui::raceDialog *ui;

    void closeEvent(QCloseEvent *event);

};

#endif // RACEDIALOG_H

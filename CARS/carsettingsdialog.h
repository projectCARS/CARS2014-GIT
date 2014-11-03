#ifndef CarSettingsDialog_H
#define CarSettingsDialog_H

#include "qtheaders.h"
#include "cargroupbox.h"

namespace Ui {
class CarSettingsDialog;
}

class CarSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CarSettingsDialog(QWidget *parent = 0);
    ~CarSettingsDialog();

private slots:
    void on_addCarButton_clicked();

    void on_removeCarButton_clicked();

private:
    Ui::CarSettingsDialog *ui;
    std::vector<CarGroupBox*> m_carGroupBoxes;
    QSettings m_settings;
    int m_numCars;

    void addNewCar(void);
    void addCarGroupBox(void);
    void saveCarSettings(void);
    void closeEvent(QCloseEvent *event);
};

#endif // CarSettingsDialog_H

#ifndef PIDSETTINGS_H
#define PIDSETTINGS_H

#include <QDialog>
#include "qtheaders.h"
#include <QSettings>

namespace Ui {
class PidSettings;
}

class PidSettings : public QDialog
{
    Q_OBJECT

public:
    explicit PidSettings(QWidget *parent = 0);
    ~PidSettings();

private slots:
    void on_pushButton_clicked();

    void on_KpDecreaseButton_clicked();

    void on_KpIncreaseButton_clicked();

    void on_KiDecreaseButton_clicked();

    void on_KiIncreaseButton_clicked();

    void on_KdDecreaseButton_clicked();

    void on_KdIncreaseButton_clicked();

    void on_KaDecreaseButton_clicked();

    void on_KaIncreaseButton_clicked();

    void on_KBrakeDecreaseButton_clicked();

    void on_KBrakeIncreaseButton_clicked();

    void savePidSettings();

    void loadPidSettings();

private:
    Ui::PidSettings *ui;
    float m_Kp = 0;
    float m_Ki = 0;
    float m_Kd = 0;
    float m_Ka = 0;
    float m_KBrake = 0;

    QSettings m_settings;

};

#endif // PIDSETTINGS_H

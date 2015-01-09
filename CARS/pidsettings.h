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

    void savePidSettings();

    void loadPidSettings();

    void on_KPlineEdit_editingFinished();

    void on_KIlineEdit_editingFinished();

    void on_KDlineEdit_editingFinished();

    void on_KAlineEdit_editingFinished();

    void on_KBRAKElineEdit_editingFinished();

    void on_KpTurnlineEdit_editingFinished();

    void on_KiTurnlineEdit_editingFinished();

    void on_KdTurnlineEdit_editingFinished();

private:
    Ui::PidSettings *ui;
    float m_Kp = 0;
    float m_Ki = 0;
    float m_Kd = 0;
    float m_Ka = 0;
    float m_KBrake = 0;
    float m_KpTurn = 0;
    float m_KiTurn = 0;
    float m_KdTurn = 0;

    QSettings m_settings;

};

#endif // PIDSETTINGS_H

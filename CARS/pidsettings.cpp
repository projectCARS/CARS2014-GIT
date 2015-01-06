#include "pidsettings.h"
#include "ui_pidsettings.h"

PidSettings::PidSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PidSettings)
{
    ui->setupUi(this);
    loadPidSettings();
    ui->kplabel->setText(QString("%1").arg(m_Kp));
    ui->kdlabel->setText(QString("%1").arg(m_Kd));
    ui->kilabel->setText(QString("%1").arg(m_Ki));
    ui->kalabel->setText(QString("%1").arg(m_Ka));
    ui->kbrakelabel->setText(QString("%1").arg(m_KBrake));

}

PidSettings::~PidSettings()
{
    delete ui;
}

void PidSettings::on_pushButton_clicked()
{
    savePidSettings();
    this->close();
}

void PidSettings::on_KpDecreaseButton_clicked()
{
    m_Kp -= 0.05f;
    ui->kplabel->setText(QString("%1").arg(m_Kp));
}

void PidSettings::on_KpIncreaseButton_clicked()
{
    m_Kp += 0.05f;
    ui->kplabel->setText(QString("%1").arg(m_Kp));
}

void PidSettings::on_KiDecreaseButton_clicked()
{
    m_Ki -= 0.05f;
    ui->kilabel->setText(QString("%1").arg(m_Ki));
}

void PidSettings::on_KiIncreaseButton_clicked()
{
    m_Ki += 0.05f;
    ui->kilabel->setText(QString("%1").arg(m_Ki));
}

void PidSettings::on_KdDecreaseButton_clicked()
{
    m_Kd -= 0.05f;
    ui->kdlabel->setText(QString("%1").arg(m_Kd));
}

void PidSettings::on_KdIncreaseButton_clicked()
{
    m_Kd += 0.05f;
    ui->kdlabel->setText(QString("%1").arg(m_Kd));
}

void PidSettings::on_KaDecreaseButton_clicked()
{
    m_Ka -= 0.05f;
    ui->kalabel->setText(QString("%1").arg(m_Ka));
}

void PidSettings::on_KaIncreaseButton_clicked()
{
    m_Ka += 0.05f;
    ui->kalabel->setText(QString("%1").arg(m_Ka));
}

void PidSettings::on_KBrakeDecreaseButton_clicked()
{
    m_KBrake -= 0.05f;
    ui->kbrakelabel->setText(QString("%1").arg(m_KBrake));
}

void PidSettings::on_KBrakeIncreaseButton_clicked()
{
    m_KBrake += 0.05f;
    ui->kbrakelabel->setText(QString("%1").arg(m_KBrake));
}

void PidSettings::savePidSettings()
{
    m_settings.setValue("pid_settings/Kp", m_Kp);
    m_settings.setValue("pid_settings/Kd", m_Kd);
    m_settings.setValue("pid_settings/Ki", m_Ki);
    m_settings.setValue("pid_settings/Ka", m_Ka);
    m_settings.setValue("pid_settings/KBrake", m_KBrake);
}

void PidSettings::loadPidSettings()
{
    m_settings.beginGroup(QString("pid_settings"));
    m_Kp = m_settings.value("Kp").toFloat();
    m_Kd = m_settings.value("Kd").toFloat();
    m_Ki = m_settings.value("Ki").toFloat();
    m_Ka = m_settings.value("Ka").toFloat();
    m_KBrake = m_settings.value("KBrake").toFloat();
    m_settings.endGroup();
}

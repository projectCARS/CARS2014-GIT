#include "pidsettings.h"
#include "ui_pidsettings.h"

PidSettings::PidSettings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PidSettings)
{
    ui->setupUi(this);
    loadPidSettings();
    setFixedSize(181,580);
    ui->KPlineEdit->setText(QString("%1").arg(m_Kp));
    ui->KDlineEdit->setText(QString("%1").arg(m_Kd));
    ui->KIlineEdit->setText(QString("%1").arg(m_Ki));
    ui->KAlineEdit->setText(QString("%1").arg(m_Ka));
    ui->KBRAKElineEdit->setText(QString("%1").arg(m_KBrake));
    ui->KpTurnlineEdit->setText(QString("%1").arg(m_KpTurn));
    ui->KiTurnlineEdit->setText(QString("%1").arg(m_KiTurn));
    ui->KdTurnlineEdit->setText(QString("%1").arg(m_KdTurn));

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

void PidSettings::savePidSettings()
{
    m_settings.setValue("pid_settings/Kp", m_Kp);
    m_settings.setValue("pid_settings/Kd", m_Kd);
    m_settings.setValue("pid_settings/Ki", m_Ki);
    m_settings.setValue("pid_settings/Ka", m_Ka);
    m_settings.setValue("pid_settings/KBrake", m_KBrake);
    m_settings.setValue("pid_settings/KpTurn", m_KpTurn);
    m_settings.setValue("pid_settings/KiTurn", m_KiTurn);
    m_settings.setValue("pid_settings/KdTurn", m_KdTurn);
}

void PidSettings::loadPidSettings()
{
    m_settings.beginGroup(QString("pid_settings"));
    m_Kp = m_settings.value("Kp").toFloat();
    m_Kd = m_settings.value("Kd").toFloat();
    m_Ki = m_settings.value("Ki").toFloat();
    m_Ka = m_settings.value("Ka").toFloat();
    m_KBrake = m_settings.value("KBrake").toFloat();
    m_KpTurn = m_settings.value("KpTurn").toFloat();
    m_KiTurn = m_settings.value("KiTurn").toFloat();
    m_KdTurn = m_settings.value("KdTurn").toFloat();
    m_settings.endGroup();
}

void PidSettings::on_KPlineEdit_editingFinished()
{
    m_Kp = ui->KPlineEdit->text().toFloat();
}

void PidSettings::on_KIlineEdit_editingFinished()
{
    m_Ki = ui->KIlineEdit->text().toFloat();
}

void PidSettings::on_KDlineEdit_editingFinished()
{
    m_Kd = ui->KDlineEdit->text().toFloat();
}

void PidSettings::on_KAlineEdit_editingFinished()
{
    m_Ka = ui->KAlineEdit->text().toFloat();
}

void PidSettings::on_KBRAKElineEdit_editingFinished()
{
    m_KBrake = ui->KBRAKElineEdit->text().toFloat();
}

void PidSettings::on_KpTurnlineEdit_editingFinished()
{
    m_KpTurn= ui->KpTurnlineEdit->text().toFloat();
}

void PidSettings::on_KiTurnlineEdit_editingFinished()
{
    m_KiTurn = ui->KiTurnlineEdit->text().toFloat();
}

void PidSettings::on_KdTurnlineEdit_editingFinished()
{
    m_KdTurn = ui->KdTurnlineEdit->text().toFloat();
}

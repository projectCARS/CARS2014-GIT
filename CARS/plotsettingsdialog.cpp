#include "plotsettingsdialog.h"
#include "ui_plotsettingsdialog.h"

plotsettingsdialog::plotsettingsdialog(QDialog *parent) :
    QDialog(parent),
    ui(new Ui::plotsettingsdialog)
{
    ui->setupUi(this);
    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));
    setFixedSize(450, 225);

    ui->referenceCheckBox->setChecked(m_settings.value("plotSettings/adGainReferencePlot").toBool());
    ui->sectionTimeCheckBox->setChecked(m_settings.value("plotSettings/adSectionTimePlot").toBool());
    ui->GainReferenceCheckBox->setChecked(m_settings.value("plotSettings/AdaptiveGainPlot").toBool());
}

plotsettingsdialog::~plotsettingsdialog()
{
    delete ui;
}

void plotsettingsdialog::on_referenceCheckBox_toggled(bool checked)
{
    m_settings.setValue("plotSettings/adGainReferencePlot", checked);
}

void plotsettingsdialog::on_sectionTimeCheckBox_toggled(bool checked)
{
    m_settings.setValue("plotSettings/adSectionTimePlot", checked);
}

void plotsettingsdialog::on_GainReferenceCheckBox_toggled(bool checked)
{
    m_settings.setValue("plotSettings/AdaptiveGainPlot", checked);
}

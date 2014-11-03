#include "drawsettingsdialog.h"
#include "ui_drawsettingsdialog.h"

#include "definitions.h"

//#include <QDebug>

DrawSettingsDialog::DrawSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DrawSettingsDialog)
{
    ui->setupUi(this);
    setFixedSize(336, 301);

    // Read background image from file.
    if (m_settings.contains("draw_settings/background"))
    {
        m_backgroundPath = m_settings.value("draw_settings/background").toString();
    }
    else
    {
        // TODO: choose a default file.
        m_backgroundPath = "";
    }

    // Update name in backgroundLabel.
    QFileInfo fileInfo(m_backgroundPath);
    ui->backgroundLabel->setText(fileInfo.fileName());

    // Number of cars read from settings file.
    m_numCars = 0;

    // Add default general draw settings if no such settings are found.
    if (!m_settings.contains("draw_settings/general_draw_settings"))
    {
        // General settings.
        m_settings.beginGroup(QString("draw_settings"));
        m_settings.setValue("general_draw_settings", (unsigned int)0);
        m_settings.endGroup();
    }

    // Read all draw settings from settings file.
    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        if (m_settings.contains(QString("draw_settings/id%1/car_specific_draw_settings").arg(m_numCars)))
        {
            // Add id to combo box.
            ui->idComboBox->addItem(QString("%1").arg(m_numCars), m_numCars);

            m_settings.beginGroup(QString("draw_settings/id%1").arg(m_numCars));
            m_carSpecificDrawSettings.push_back(m_settings.value("car_specific_draw_settings").toUInt());
            m_settings.endGroup();
        }
        else
        {
            m_settings.beginGroup(QString("draw_settings/id%1").arg(m_numCars));
            m_settings.setValue("car_specific_draw_settings", (unsigned int)0);
            m_settings.endGroup();

            ui->idComboBox->addItem(QString("%1").arg(m_numCars), m_numCars);
            m_settings.beginGroup(QString("draw_settings/id%1").arg(m_numCars));
            m_carSpecificDrawSettings.push_back(m_settings.value("car_specific_draw_settings").toUInt());
            m_settings.endGroup();
        }

        m_numCars++;
    }

    // Update general draw settings in GUI.
    m_settings.beginGroup(QString("draw_settings"));
    ui->referenceCheckBox->setChecked(m_settings.value("general_draw_settings").toUInt() & GeneralDrawSettings::Reference);
    ui->animationsCheckBox->setChecked(m_settings.value("general_draw_settings").toUInt() & GeneralDrawSettings::Animations);
    m_settings.endGroup();

    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));
    connect(ui->idComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSettings(int)));

    // Set current car id to 0, and update checkboxes.
    ui->idComboBox->setCurrentIndex(0);
    updateSettings(0);
}

DrawSettingsDialog::~DrawSettingsDialog()
{
    saveDrawSettings();
    m_settings.sync();
    delete ui;
}

void DrawSettingsDialog::updateSettings(int id)
{
    ui->pathCheckBox->setChecked(m_carSpecificDrawSettings[id] & CarSpecificDrawSettings::Path);
    ui->circleCheckBox->setChecked(m_carSpecificDrawSettings[id] & CarSpecificDrawSettings::Circle);
    ui->rectangleCheckBox->setChecked(m_carSpecificDrawSettings[id] & CarSpecificDrawSettings::Rectangle);
    ui->carTracksCheckBox->setChecked(m_carSpecificDrawSettings[id] & CarSpecificDrawSettings::CarTracks);
}

void DrawSettingsDialog::saveDrawSettings(void)
{
    // Extract current state of general draw settings.
    unsigned int generalDrawSettings =
        ui->referenceCheckBox->isChecked()*GeneralDrawSettings::Reference |
        ui->animationsCheckBox->isChecked()*GeneralDrawSettings::Animations;
    // General settings.
    m_settings.beginGroup(QString("draw_settings"));
    m_settings.setValue("general_draw_settings", (unsigned int)generalDrawSettings);
    m_settings.setValue("background", m_backgroundPath);
    m_settings.endGroup();

    for (int i=0; i<m_numCars; i++)
    {
        // Car specific settings.
        m_settings.beginGroup(QString("draw_settings/id%1").arg(i));
        m_settings.setValue("car_specific_draw_settings", (unsigned int)m_carSpecificDrawSettings[i]);
        m_settings.endGroup();
    }
}

void DrawSettingsDialog::on_pathCheckBox_clicked()
{
    if (ui->pathCheckBox->isChecked())
    {
        // Set the bit if check box is checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] |= CarSpecificDrawSettings::Path;
    }
    else
    {
        // Clear the bit if check box is not checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] &= ~CarSpecificDrawSettings::Path;
    }
}

void DrawSettingsDialog::on_circleCheckBox_clicked()
{
    if (ui->circleCheckBox->isChecked())
    {
        // Set the bit if check box is checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] |= CarSpecificDrawSettings::Circle;
    }
    else
    {
        // Clear the bit if check box is not checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] &= ~CarSpecificDrawSettings::Circle;
    }
}

void DrawSettingsDialog::on_rectangleCheckBox_clicked()
{
    if (ui->rectangleCheckBox->isChecked())
    {
        // Set the bit if check box is checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] |= CarSpecificDrawSettings::Rectangle;
    }
    else
    {
        // Clear the bit if check box is not checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] &= ~CarSpecificDrawSettings::Rectangle;
    }
}

void DrawSettingsDialog::on_carTracksCheckBox_clicked()
{
    if (ui->carTracksCheckBox->isChecked())
    {
        // Set the bit if check box is checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] |= CarSpecificDrawSettings::CarTracks;
    }
    else
    {
        // Clear the bit if check box is not checked.
        m_carSpecificDrawSettings[ui->idComboBox->currentIndex()] &= ~CarSpecificDrawSettings::CarTracks;
    }
}

void DrawSettingsDialog::on_chooseBackgroundButton_clicked()
{
    // This sometimes produces a strange output to the console.
    QString backgroundPath = QFileDialog::getOpenFileName(this, tr("Choose Background Image"), "indata", tr("Background Files (*.jpg *.png)"));
    if (!backgroundPath.isEmpty())
    {
        m_backgroundPath = backgroundPath;
    }
    // Update name in backgroundLabel.
    QFileInfo fileInfo(m_backgroundPath);
    ui->backgroundLabel->setText(fileInfo.fileName());
}

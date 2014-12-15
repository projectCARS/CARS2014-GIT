#include "CarSettingsDialog.h"
#include "ui_CarSettingsDialog.h"

CarSettingsDialog::CarSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CarSettingsDialog)
{
    ui->setupUi(this);
    ui->scrollAreaContentsLayout->setAlignment(Qt::AlignTop);
    // Set window size.
    setFixedSize(720,450);

    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));

    // Number of cars read from settings file.
    m_numCars = 0;

    // Read all cars from settings file.
    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        addCarGroupBox();
    }

    /* If the settings file does not contain the car with ID 0 (i.e. it contains no car whatsoever),
    add a car with default attributes. */
    if (!m_settings.contains("car/id0/mode"))
    {
        addNewCar();
    }
}

CarSettingsDialog::~CarSettingsDialog()
{
    m_settings.sync();
    delete ui;
}

void CarSettingsDialog::on_addCarButton_clicked()
{
    addNewCar();
}

void CarSettingsDialog::on_removeCarButton_clicked()
{
    m_settings.remove(QString("car/id%1").arg(m_numCars-1));
    /* Remove element m_numCars-1 (i.e. the last element in the vector). Note that the CarGroupBox
     * object is deleted when the CarSettingsDialog object is deleted. */
    m_carGroupBoxes.back()->close();
    m_carGroupBoxes.pop_back();
    m_numCars--;

    // Prevent user from removing last car.
    if (m_numCars == 1)
    {
        ui->removeCarButton->setEnabled(false);
    }

    //updateGeometry();
}


void CarSettingsDialog::addNewCar(void)
{
    // Create settings for a car with default values.
    m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
    m_settings.setValue("mode",(int)CarMode::NotConnected);
    m_settings.setValue("filter",(int)FilterType::EKF);
    m_settings.setValue("motion_model",(int)MotionModelType::CTModel);
    m_settings.setValue("handController",(int)HandController::HandControl_1);
    m_settings.setValue("controller",(int)ControllerType::PIDController);
    m_settings.endGroup();
    addCarGroupBox();
}

void CarSettingsDialog::addCarGroupBox()
{
    m_carGroupBoxes.push_back(new CarGroupBox(ui->scrollAreaWidgetContents));
    ui->scrollAreaContentsLayout->addWidget(m_carGroupBoxes.back());

    m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
    m_carGroupBoxes[m_numCars]->setId(m_numCars);
    m_carGroupBoxes[m_numCars]->setMode(m_settings.value("mode").toInt());
    m_carGroupBoxes[m_numCars]->setFilterType(m_settings.value("filter").toInt());
    m_carGroupBoxes[m_numCars]->setMotionModelType(m_settings.value("motion_model").toInt());
    m_carGroupBoxes[m_numCars]->setHandControllerType(m_settings.value("handController").toInt());
    m_carGroupBoxes[m_numCars]->setControllerType(m_settings.value("controller").toInt());
    m_settings.endGroup();
    m_numCars++;

    // Enable removeCarButton if there are two cars.
    if (m_numCars == 2)
    {
        ui->removeCarButton->setEnabled(true);
    }

    //updateGeometry();
}

void CarSettingsDialog::closeEvent(QCloseEvent *event)
{
    saveCarSettings();
    event->accept();
}

void CarSettingsDialog::saveCarSettings(void)
{
    for (int id=0; id<m_numCars; id++)
    {
        m_settings.beginGroup(QString("car/id%1").arg(id));
        m_settings.setValue("mode", m_carGroupBoxes[id]->getMode());
        m_settings.setValue("filter", m_carGroupBoxes[id]->getFilterType());
        m_settings.setValue("handController", m_carGroupBoxes[id]->getHandControllerType());
        m_settings.setValue("motion_model", m_carGroupBoxes[id]->getMotionModelType());
        m_settings.setValue("controller", m_carGroupBoxes[id]->getControllerType());
        m_settings.endGroup();
    }
}


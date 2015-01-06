#include "cargroupbox.h"
#include "ui_cargroupbox.h"

CarGroupBox::CarGroupBox(QWidget *parent) :
    QGroupBox(parent),
    ui(new Ui::CarGroupBox)
{
    ui->setupUi(this);

    // Create a button group and set to exclusive.
    m_buttonGroup = new QButtonGroup(this);
    m_buttonGroup->addButton(ui->autoRadioButton, CarMode::Auto);
    m_buttonGroup->addButton(ui->assistedRadioButton, CarMode::Assisted);
    m_buttonGroup->addButton(ui->manualRadioButton, CarMode::Manual);
    m_buttonGroup->addButton(ui->notConnectedRadioButton, CarMode::NotConnected);
    m_buttonGroup->setExclusive(true);

    // Add filters.
    ui->filterComboBox->addItem("EKF", (int)FilterType::EKF);
    ui->filterComboBox->addItem("ParticleFilter", (int)FilterType::ParticleFilter);
    ui->filterComboBox->addItem("NoFilter", (int)FilterType::NoFilter);
    // Add motion models.
    ui->motionModelComboBox->addItem("CTModel", (int)MotionModelType::CTModel);
    ui->motionModelComboBox->addItem("STModel", (int)MotionModelType::STModel);

    // Add hand controllers
    ui->handControllercomboBox->addItem("Hand Controller 1", (int)HandController::HandControl_1);
    ui->handControllercomboBox->addItem("Hand Controller 2", (int)HandController::HandControl_2);

    // Add controllers.
    ui->controllerComboBox->addItem("PIDdefault", (int)ControllerType::PIDdefault);
    ui->controllerComboBox->addItem("PIDaggressive", (int)ControllerType::PIDaggressive);
    ui->controllerComboBox->addItem("PIDadaptiveGain", (int)ControllerType::PIDadaptiveGain);
    ui->controllerComboBox->addItem("PIDadaptiveSection", (int)ControllerType::PIDadaptiveSection);



    // Set fixed width of combo boxes (not optimal solution, but it works).
    ui->filterComboBox->setFixedWidth(70);
    ui->motionModelComboBox->setFixedWidth(100);
    ui->controllerComboBox->setFixedWidth(130);

    QSizePolicy policy = sizePolicy();
    policy.setHorizontalPolicy(QSizePolicy::Fixed);
    policy.setVerticalPolicy(QSizePolicy::Fixed);
    setSizePolicy(policy);
}

CarGroupBox::~CarGroupBox()
{
    delete ui;
}

void CarGroupBox::setId(int id)
{
    ui->idLabel->setText(QString("%1").arg(id));
}

int CarGroupBox::getId()
{
    bool succeeded;
    int id = ui->idLabel->text().toInt(&succeeded,10);

    return id;
}

void CarGroupBox::setMode(int mode)
{
    switch (mode)
    {
        case CarMode::Manual:
            ui->manualRadioButton->setChecked(true);
            break;
        case CarMode::Auto:
            ui->autoRadioButton->setChecked(true);
            break;
        case CarMode::Assisted:
            ui->assistedRadioButton->setChecked(true);
            break;
        case CarMode::NotConnected:
            ui->notConnectedRadioButton->setChecked(true);
            break;
        default:
            qDebug("Error: default label reached in setMode(), cargroupbox.cpp.");
    }
}

int CarGroupBox::getMode()
{
    return m_buttonGroup->checkedId();
}

void CarGroupBox::setFilterType(int type)
{
    ui->filterComboBox->setCurrentIndex(ui->filterComboBox->findData((int)type));
}

int CarGroupBox::getFilterType()
{
    return ui->filterComboBox->itemData(ui->filterComboBox->currentIndex()).toInt();
}

void CarGroupBox::setMotionModelType(int type)
{
    ui->motionModelComboBox->setCurrentIndex(ui->motionModelComboBox->findData((int)type));
}

int CarGroupBox::getMotionModelType()
{
    return ui->motionModelComboBox->itemData(ui->motionModelComboBox->currentIndex()).toInt();
}

void CarGroupBox::setHandControllerType(int type)
{
    ui->handControllercomboBox->setCurrentIndex(ui->handControllercomboBox->findData((int)type));
}

int CarGroupBox::getHandControllerType()
{
    return ui->handControllercomboBox->itemData(ui->handControllercomboBox->currentIndex()).toInt();
}

void CarGroupBox::setControllerType(int type)
{
    ui->controllerComboBox->setCurrentIndex(ui->controllerComboBox->findData((int)type));
}

int CarGroupBox::getControllerType()
{
    return ui->controllerComboBox->itemData(ui->controllerComboBox->currentIndex()).toInt();
}

void CarGroupBox::on_manualRadioButton_toggled(bool checked)
{
    ui->filterComboBox->setEnabled(true);
    ui->motionModelComboBox->setEnabled(true);
    ui->controllerComboBox->setEnabled(!checked);
}

void CarGroupBox::on_notConnectedRadioButton_toggled(bool checked)
{
    ui->controllerComboBox->setEnabled(!checked);
}

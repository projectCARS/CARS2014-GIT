#include "racegroupbox.h"
#include "ui_racegroupbox.h"

RaceGroupBox::RaceGroupBox(QWidget *parent, int idNumber) :
    QGroupBox(parent),
    ui(new Ui::RaceGroupBox)
{
    ui->setupUi(this);

    m_buttonGroup = new QButtonGroup(this);
    m_buttonGroup->addButton(ui->RaceCheckBox, 1);
    m_buttonGroup->setExclusive(false);

    // Create a button group and set to exclusive.
    setId(idNumber);
    QSizePolicy policy = sizePolicy();
    policy.setHorizontalPolicy(QSizePolicy::Preferred);
    policy.setVerticalPolicy(QSizePolicy::Preferred);
    setSizePolicy(policy);
    setRaceCheckBox();
}

RaceGroupBox::~RaceGroupBox()
{
    delete ui;
}

void RaceGroupBox::setId(int id)
{
    ui->idLabel->setText(QString("%1").arg(id));
    this->id = id;
}

int RaceGroupBox::getId()
{
    return id;
}

void RaceGroupBox::setRaceCheckBox()
{
    std::cout << getId() << std::endl;
    m_settings.beginGroup(QString("race_settings/id%1").arg(getId()));
    if(m_settings.value("race").toInt() == 1)
        ui->RaceCheckBox->setChecked(true);
    else
        ui->RaceCheckBox->setChecked(false);
    m_settings.endGroup();
}

void RaceGroupBox::on_RaceCheckBox_clicked()
{
    if (ui->RaceCheckBox->isChecked())
    {
        m_settings.beginGroup(QString("race_settings/id%1").arg(getId()));
        m_settings.setValue("race", 1);
        m_settings.endGroup();
    }
    else
    {
        m_settings.beginGroup(QString("race_settings/id%1").arg(getId()));
        m_settings.setValue("race", 0);
        m_settings.endGroup();
    }
}

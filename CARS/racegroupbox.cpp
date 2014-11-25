#include "racegroupbox.h"
#include "ui_racegroupbox.h"

RaceGroupBox::RaceGroupBox(QWidget *parent) :
    QGroupBox(parent),
    ui(new Ui::RaceGroupBox)
{
    ui->setupUi(this);

    m_buttonGroup = new QButtonGroup(this);
    m_buttonGroup->addButton(ui->radioButton, 1);

    // Create a button group and set to exclusive.

    QSizePolicy policy = sizePolicy();
    policy.setHorizontalPolicy(QSizePolicy::Preferred);
    policy.setVerticalPolicy(QSizePolicy::Preferred);
    setSizePolicy(policy);
}

RaceGroupBox::~RaceGroupBox()
{
    delete ui;
}

void RaceGroupBox::setId(int id)
{
    ui->idLabel->setText(QString("%1").arg(id));
}

int RaceGroupBox::getId()
{
    bool succeeded;
    int id = ui->idLabel->text().toInt(&succeeded,10);
    return id;
}

#include "racedialog.h"
#include "ui_racedialog.h"

#include "racegroupbox.h"

#include "definitions.h"

#include <QDebug>


raceDialog::raceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::raceDialog)
{
    ui->setupUi(this);
    //setFixedSize(540,450);
    ui->scrollAreaContentsLayout->setAlignment(Qt::AlignTop);

    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));

    m_numCars = 0;

    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        //addRaceGroupBox();
    }
}

raceDialog::~raceDialog()
{
    delete ui;
}

void raceDialog::closeEvent(QCloseEvent *event)
{
    event->accept();
}

void raceDialog::on_startRaceButton_released()
{
    int ans = QMessageBox::question(this,tr("Ready?"),
                                    tr("Place your cars in starting positions,\nthen click 'Yes' for count down "),
                                    QMessageBox::Cancel | QMessageBox::Yes,
                                    QMessageBox::Yes);
    switch (ans)
    {
    case (QMessageBox::Yes):
        close();    //close racedialog
        //start count down
        //start race //make race class?


        break;
    default:
        ;
    }
}

void raceDialog::on_carSettingsButton_clicked()
{
    CarSettingsDialog dialog;
    dialog.setModal(true);
    dialog.exec();
}

void raceDialog::addRaceGroupBox()
{
    m_raceGroupBoxes.push_back(new RaceGroupBox(ui->scrollAreaWidgetContents));
    ui->scrollAreaContentsLayout->addWidget(m_raceGroupBoxes.back());

    m_settings.beginGroup(QString("car/id%1").arg(m_numCars));
    m_raceGroupBoxes[m_numCars]->setId(m_numCars);
    m_settings.endGroup();
    m_numCars++;

    // Enable removeCarButton if there are two cars.
}

#include "racedialog.h"
#include "ui_racedialog.h"
#include "mainwindow.h"

#include "racegroupbox.h"

#include "definitions.h"

#include <QDebug>


raceDialog::raceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::raceDialog)
{
    lapNumber = m_settings.value("race_settings/number_of_laps").toInt();
    ui->setupUi(this);
    ui->scrollAreaContentsLayout->setAlignment(Qt::AlignTop);
    ui->lapNumberLabel->setText(QString("%1").arg(lapNumber));

    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));

    m_numCars = 0;

    while (m_settings.contains(QString("car/id%1/mode").arg(m_numCars)))
    {
        addRaceGroupBox();
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

void raceDialog::on_decreaseLapsButton_clicked()
{
    if(lapNumber > 1)
        lapNumber--;
    ui->lapNumberLabel->setText(QString("%1").arg(lapNumber));
    saveRaceSettings();
}

void raceDialog::on_increaseLapsButton_clicked()
{
    if(lapNumber < 99)
        lapNumber++;
    ui->lapNumberLabel->setText(QString("%1").arg(lapNumber));
    saveRaceSettings();
}


void raceDialog::addRaceGroupBox()
{
    m_raceGroupBoxes.push_back(new RaceGroupBox(ui->scrollAreaWidgetContents, m_numCars));
    ui->scrollAreaContentsLayout->addWidget(m_raceGroupBoxes.back());
    //m_raceGroupBoxes[m_numCars]->setId(m_numCars);
    m_numCars++;

    // Enable removeCarButton if there are two cars.
}

void raceDialog::saveRaceSettings(void)
{
    m_settings.beginGroup(QString("race_settings"));
    m_settings.setValue("number_of_laps", lapNumber);
    m_settings.endGroup();

}

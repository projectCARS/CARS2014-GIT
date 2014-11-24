#include "racedialog.h"
#include "ui_racedialog.h"


#include "definitions.h"


raceDialog::raceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::raceDialog)
{
    ui->setupUi(this);
    //setFixedSize(540,450);

    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));
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

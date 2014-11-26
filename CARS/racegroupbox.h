#ifndef RACEGROUPBOX_H
#define RACEGROUPBOX_H

#include "qtheaders.h"
#include "definitions.h"

namespace Ui {
class RaceGroupBox;
}

class RaceGroupBox : public QGroupBox
{
    Q_OBJECT

public:
    explicit RaceGroupBox(QWidget *parent = 0, int idNumber = 0);
    ~RaceGroupBox();

    void setId(int id);
    int getId();
    void setRaceCheckBox();

private slots:


    void on_RaceCheckBox_clicked();

private:
    int id;
    Ui::RaceGroupBox *ui;
    QButtonGroup *m_buttonGroup;
    QSettings m_settings;
};

#endif // RACEGROUPBOX_H

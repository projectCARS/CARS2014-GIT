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
    explicit RaceGroupBox(QWidget *parent = 0);
    ~RaceGroupBox();

    void setId(int id);
    int getId();

private slots:


private:
    Ui::RaceGroupBox *ui;
    QButtonGroup *m_buttonGroup;
};

#endif // RACEGROUPBOX_H

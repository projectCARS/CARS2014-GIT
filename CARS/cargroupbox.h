#ifndef CARGROUPBOX_H
#define CARGROUPBOX_H

#include "qtheaders.h"
#include "definitions.h"

namespace Ui {
class CarGroupBox;
}

class CarGroupBox : public QGroupBox
{
    Q_OBJECT

public:
    explicit CarGroupBox(QWidget *parent = 0);
    ~CarGroupBox();

    void setId(int id);
    int getId();

    void setMode(int mode);
    int getMode();

    void setFilterType(int type);
    int getFilterType();

    void setMotionModelType(int type);
    int getMotionModelType();

    void setControllerType(int type);
    int getControllerType();

private slots:

    void on_manualRadioButton_toggled(bool checked);

    void on_notConnectedRadioButton_toggled(bool checked);

private:
    Ui::CarGroupBox *ui;
    QButtonGroup *m_buttonGroup;
};

#endif // CARGROUPBOX_H

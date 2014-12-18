#ifndef PLOTSETTINGSDIALOG_H
#define PLOTSETTINGSDIALOG_H

#include "qtheaders.h"
#include "definitions.h"

namespace Ui {
class plotsettingsdialog;
}

class plotsettingsdialog : public QDialog
{
    Q_OBJECT

public:
    explicit plotsettingsdialog(QDialog *parent = 0);
    ~plotsettingsdialog();

private slots:

    void on_referenceCheckBox_toggled(bool checked);

    void on_sectionTimeCheckBox_toggled(bool checked);

    void on_speedErrorCheckBox_toggled(bool checked);

private:
    Ui::plotsettingsdialog *ui;
    QSettings m_settings;
};

#endif // PLOTSETTINGSDIALOG_H

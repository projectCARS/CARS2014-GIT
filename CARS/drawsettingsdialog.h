#ifndef DRAWSETTINGSDIALOG_H
#define DRAWSETTINGSDIALOG_H

#include "qtheaders.h"

namespace Ui {
class DrawSettingsDialog;
}

class DrawSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DrawSettingsDialog(QWidget *parent = 0);
    ~DrawSettingsDialog();

private slots:
    void on_pathCheckBox_clicked();
    void on_circleCheckBox_clicked();
    void on_rectangleCheckBox_clicked();
    void on_timerCheckBox_clicked();
    void updateSettings(int id);

    void on_carTracksCheckBox_clicked();

    void on_chooseBackgroundButton_clicked();

private:
    Ui::DrawSettingsDialog *ui;
    QSettings m_settings;
    int m_numCars;
    std::vector<unsigned int> m_carSpecificDrawSettings;
    QString m_backgroundPath;

    void saveDrawSettings(void);
};

#endif // DRAWSETTINGSDIALOG_H

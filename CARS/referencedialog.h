#ifndef REFERENCEDIALOG_H
#define REFERENCEDIALOG_H

#include "qtheaders.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Ui {
class ReferenceDialog;
}

class ReferenceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ReferenceDialog(QWidget *parent = 0);
    ~ReferenceDialog();

private:
    Ui::ReferenceDialog *ui;

    QString m_fileName;
    cv::Mat m_image;
    QSettings m_settings;
    std::vector<float> m_ref;
    std::vector<float> m_vRef;
    int m_refLen;
    double m_gain = 0;
    double m_offset = 0;

    void loadReference(void);
    void drawReference(void);
    void displayReference(void);
    QImage ReferenceDialog::matToQImage(const cv::Mat &mat);

private slots:
    void on_chooseFileButton_clicked();

    void on_reversecheckBox_clicked(bool checked);
    void on_decreaseGainButton_clicked();
    void on_increaseGainButton_clicked();
    void on_decreaseOffsetButton_clicked();
    void on_increaseOffsetButton_clicked();
};

#endif // REFERENCEDIALOG_H

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

    void loadReference(void);
    void drawReference(void);
    void displayReference(void);
    QImage ReferenceDialog::matToQImage(const cv::Mat &mat);

private slots:
    void on_chooseFileButton_clicked();

    void on_reversecheckBox_clicked(bool checked);
};

#endif // REFERENCEDIALOG_H

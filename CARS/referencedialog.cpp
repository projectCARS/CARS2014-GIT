#include "referencedialog.h"
#include "ui_referencedialog.h"

#include <fstream>

ReferenceDialog::ReferenceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ReferenceDialog)
{
    ui->setupUi(this);

    setFixedSize(620,550);
    ui->imageLabel->setFixedSize(582,464);
    ui->gainLabel->setFixedWidth(20);
    ui->offsetLabel->setFixedWidth(20);
    ui->decreaseGainButton->setFixedWidth(25);
    ui->increaseGainButton->setFixedWidth(25);
    ui->decreaseOffsetButton->setFixedWidth(25);
    ui->increaseOffsetButton->setFixedWidth(25);

    if (m_settings.contains("reference/file_name"))
    {
        m_fileName = m_settings.value("reference/file_name").toString();
    }
    else
    {
        // TODO: choose a default file.
        m_fileName = "";
    }
    m_gain = m_settings.value("reference/gain").toDouble();
    m_offset = m_settings.value("reference/offset").toDouble();

    QFileInfo fileInfo(m_fileName);
    ui->nameLabel->setText(fileInfo.fileName());
    ui->reversecheckBox->setChecked(m_settings.value("reference/reverse").toBool());
    ui->gainLabel->setText(m_settings.value("reference/gain").toString());
    ui->offsetLabel->setText(m_settings.value("reference/offset").toString());

    connect(ui->closeButton, SIGNAL(clicked()), this, SLOT(close()));

    // Display track with reference curve.
    displayReference();
}

ReferenceDialog::~ReferenceDialog()
{
    // Save settings.
    m_settings.beginGroup("reference");
    m_settings.setValue("file_name", m_fileName);
    m_settings.endGroup();
    m_settings.sync();

    delete ui;
}

void ReferenceDialog::on_chooseFileButton_clicked()
{
    // This sometimes produces a strange output to the console.
    QString fileName = QFileDialog::getOpenFileName(this, tr("Choose Reference Curve"), "indata/reference", tr("Reference Files (*.txt)"));
    if (!fileName.isEmpty())
    {
        m_fileName = fileName;
    }
    //QFileInfo fileInfo(m_fileName);
    displayReference();
}

void ReferenceDialog::displayReference(void)
{
    // Display file name.
    if (m_fileName != "")
    {
        QFileInfo fileInfo(m_fileName);
        ui->nameLabel->setText(fileInfo.fileName());
    }
    m_image = cv::imread("indata/worldMap.png", CV_LOAD_IMAGE_COLOR);
    // Load the reference curve into m_ref (in pixel coordinates). This is done just to be able to draw the curve to m_image.
    loadReference();
    // Draw the reference curve.
    drawReference();
    // Convert to QImage.
    QImage qImage  = matToQImage(m_image);
    // Display current reference curve.
    ui->imageLabel->setPixmap(QPixmap::fromImage(qImage).scaled(ui->imageLabel->width(), ui->imageLabel->height(), Qt::KeepAspectRatio));
}

QImage ReferenceDialog::matToQImage(const cv::Mat &mat)
{
    if(mat.type() == CV_8UC1)
    {
        QVector<QRgb> colorTable;
        for (int i=0; i<256; i++)
        {
            colorTable.push_back(qRgb(i,i,i));
        }
        const uchar *qImageBuffer = (const uchar*)mat.data;
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        return img.rgbSwapped();
    }
    else if (mat.type() == CV_8UC3)
    {
        const uchar *qImageBuffer = (const uchar*)mat.data;
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return img.rgbSwapped();
    }
    else
    {
        qDebug() << "Error: cv::Mat could not be converted to QImage.";
        return QImage();
    }
}

void ReferenceDialog::loadReference()
{	// Counter that counts the number of points on the reference curve.
    int numPoints = 0;

    // Open file with reference curve.
    std::ifstream file;
    file.open(m_fileName.toStdString().c_str(), std::ios::in);
    if (!file)
    {
        std::cout << "Error: Could not open reference file, in loadReference(), referencedialog.cpp" << std::endl;
        return;
    }
    // First row of reference.txt must be the number of reference points.
    file >> m_refLen;
    // Delete old reference vector.
    m_ref.resize(0);
    m_vRef.resize(0);
    // Allocate memory for vector with reference curve.
    m_ref.resize(m_refLen * 2);
    m_vRef.resize(m_refLen);
    // Get values from reference curve and convert from pixels to meters.
    for (int i = 0; i < m_refLen; i++)
    {
        // x pixel coordinate.
        file >> m_ref[i * 2];
        m_ref[i * 2] = m_ref[i * 2]*600;
        // y pixel coordinate.
        file >> m_ref[i * 2 + 1];
        m_ref[i * 2 + 1] = m_ref[i * 2 + 1]*600;
        // extract speed reference. NOT USED in this class
        file >> m_vRef[i];
        numPoints++;
    }

    // Check that there were at least gRefLen numbers to read.
    if (numPoints < m_refLen)
    {
        std::cout << "Error: Number of points on reference curve is smaller than m_refLen (reference file is not formatted properly), in loadReference(), referencedialog.cpp" << std::endl;
    }

    // Close file.
    file.close();
}

void ReferenceDialog::drawReference(void)
{
    for (int i = 0; i < 2*m_refLen - 2; i = i + 2)
    {
        line(m_image, cv::Point(m_ref[i], m_ref[i + 1]),
                    cv::Point(m_ref[i + 2], m_ref[i + 3]), cv::Scalar(0, 0, 255), 3);

    }
    // Draw a line between the first and last points.
    if (m_refLen > 0)
    {
        line(m_image, cv::Point(m_ref[0], m_ref[1]),
                    cv::Point(m_ref[2*m_refLen-4], m_ref[2*m_refLen-3]), cv::Scalar(0, 0, 255), 3);
    }
}

void ReferenceDialog::on_reversecheckBox_clicked(bool checked)
{
    m_settings.setValue("reference/reverse",(int)checked);
}

void ReferenceDialog::on_decreaseGainButton_clicked()
{
    if(m_gain > 0.1)
        m_gain -= 0.05;
    ui->gainLabel->setText(QString("%1").arg(m_gain));
    m_settings.setValue("reference/gain", m_gain);
}

void ReferenceDialog::on_increaseGainButton_clicked()
{
    if(m_gain < 3)
        m_gain += 0.05;
    ui->gainLabel->setText(QString("%1").arg(m_gain));
    m_settings.setValue("reference/gain", m_gain);
}

void ReferenceDialog::on_decreaseOffsetButton_clicked()
{
    if(m_offset > -0.9)
        m_offset -= 0.05;
    if(m_offset < 0.05 && m_offset > -0.05)
        m_offset = 0;
    ui->offsetLabel->setText(QString("%1").arg(m_offset));
    m_settings.setValue("reference/offset", m_offset);
}

void ReferenceDialog::on_increaseOffsetButton_clicked()
{
    if(m_offset < 0.9)
        m_offset += 0.05;
    if(m_offset < 0.05 && m_offset > -0.05)
        m_offset = 0;
    ui->offsetLabel->setText(QString("%1").arg(m_offset));
    m_settings.setValue("reference/offset", m_offset);
}

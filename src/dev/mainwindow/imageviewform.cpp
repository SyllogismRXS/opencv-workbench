/****************************************************************************
 **
 ** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
 ** Contact: http://www.qt-project.org/legal
 **
 ** This file is part of the examples of the Qt Toolkit.
 **
 ** $QT_BEGIN_LICENSE:BSD$
 ** You may use this file under the terms of the BSD license as follows:
 **
 ** "Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **   * Redistributions of source code must retain the above copyright
 **     notice, this list of conditions and the following disclaimer.
 **   * Redistributions in binary form must reproduce the above copyright
 **     notice, this list of conditions and the following disclaimer in
 **     the documentation and/or other materials provided with the
 **     distribution.
 **   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
 **     of its contributors may be used to endorse or promote products derived
 **     from this software without specific prior written permission.
 **
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 **
 ** $QT_END_LICENSE$
 **
 ****************************************************************************/

#include <QtGui>
#include <QResource>

#include "imageviewform.h"

using std::cout;
using std::endl;

ImageViewForm::ImageViewForm(QMainWindow *parent)
     : QMainWindow(parent)
{
     ui.setupUi(this);

     QResource res(":/resources/emacs.jpg");

     //cout << res.absoluteFilePath() << endl;

     //cv::Mat cv_image;
     //cv_image = cv::imread(res.absoluteFilePath(), CV_LOAD_IMAGE_COLOR);

     //cv::imshow("image", cv_image);
     
     //QImage image(":/resources/emacs.jpg");
     //
     //ui.image_frame->setPixmap(QPixmap::fromImage(image));
     //ui.image_frame->adjustSize();

     //connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(&ImageViewForm::menu_about()));
     
     connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(open()));
     connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(save()));
     connect(ui.actionQuit, SIGNAL(triggered()), this, SLOT(close()));

     connect(ui.actionAbout, SIGNAL(triggered()), this, SLOT(about()));

     timer_ = new QTimer(this);
     timer_->setInterval(10);
     timer_->start();

     connect(timer_, SIGNAL(timeout()), this, SLOT(timer_loop()));
}

void ImageViewForm::timer_loop()
{
     if ( !cv_image.empty() ) {

          if (media_type == mp4 || media_type == wmv) {
               if (vcap.isOpened()) {
                    vcap.read(cv_image);
                    show_image(cv_image);
               }
          }

          //cv::Mat temp = cv::Mat::zeros(cv_image.rows, cv_image.cols, cv_image.type());          
          //cv::addWeighted( cv_image, 0.5, temp, 0.5, 0.0, cv_image);
          //show_image(cv_image);
     }
}

void ImageViewForm::show_image(const cv::Mat &img)
{
     q_image = Mat2QImage(cv_image);
     ui.image_frame->setPixmap(QPixmap::fromImage(q_image));
     ui.image_frame->adjustSize();
}

void ImageViewForm::open()
{
     QString fileName = QFileDialog::getOpenFileName(this,
                                                     tr("Open File"), QDir::currentPath());
     if (!fileName.isEmpty()) {
          std::string fn = fileName.toStdString();

          if (fn.substr(fn.find_last_of(".") + 1) == "son") {
               media_type = sonar;               
          } else if (fn.substr(fn.find_last_of(".") + 1) == "avi") {
               media_type = avi;
               vcap.open(fn);
               vcap.read(cv_image);
               show_image(cv_image);
          } else if (fn.substr(fn.find_last_of(".") + 1) == "mp4") {
               media_type = mp4;
               vcap.open(fn);
               if (vcap.isOpened()) {
                    vcap.read(cv_image);
                    show_image(cv_image);
               }
          } else if (fn.substr(fn.find_last_of(".") + 1) == "wmv") {
               media_type = wmv;
               vcap.open(fn);
               if (vcap.isOpened()) {
                    vcap.read(cv_image);
                    show_image(cv_image);
               }
          } else {
               media_type = img;
               cv_image = cv::imread(fileName.toStdString(), CV_LOAD_IMAGE_COLOR);
               show_image(cv_image);
          }
     }     
}

void ImageViewForm::save()
{
     cout << "Save!" << endl;
}

void ImageViewForm::about()
{
     QMessageBox::about(this, tr("About The OpenCV Workbench"),
                        tr("<p><b>The OpenCV Workbench</b> is a GUI application that simplifies the computer vision prototyping process.</p><p>Author: Kevin DeMarco (kevin.demarco@gmail.com)</p>"));
}

//QImage *img = new QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
QImage ImageViewForm::Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp; // make the same cv::Mat
     cvtColor(src, temp,CV_BGR2RGB); // cvtColor Makes a copt, that what i need
     QImage dest((uchar*) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     QImage dest2(dest);
     dest2.detach(); // enforce deep copy
     return dest2;
}

cv::Mat ImageViewForm::QImage2Mat(QImage const& src)
{
     cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
     cv::Mat result; // deep copy just in case (my lack of knowledge with open cv)
     cvtColor(tmp, result,CV_BGR2RGB);
     return result;
}

//void ImageViewForm::on_inputSpinBox1_valueChanged(int value)
//{
//     //ui.outputWidget->setText(QString::number(value + ui.inputSpinBox2->value()));
//}
//
//void ImageViewForm::on_inputSpinBox2_valueChanged(int value)
//{
//     //ui.outputWidget->setText(QString::number(value + ui.inputSpinBox1->value()));
//}

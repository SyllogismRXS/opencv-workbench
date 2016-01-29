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
#include <QSettings>

#include "ObjectName.h"

using std::cout;
using std::endl;

ObjectName::ObjectName(QWidget *parent)
     : QWidget(parent)
{
     ui.setupUi(this);

     readSettings();

     // Create model
     model = new QStringListModel(this);     

     // Populate our model
     model->setStringList(list_);

     // Glue model and view together
     ui.listView->setModel(model);     
          
     connect(ui.button_add, SIGNAL(released()), this, SLOT(add_name()));
     connect(ui.button_delete, SIGNAL(released()), this, SLOT(delete_name()));
}

QString ObjectName::selected_name()
{
     QString result = "";
     QModelIndexList list = ui.listView->selectionModel()->selectedIndexes();     
     foreach(const QModelIndex &index, list){          
          result = index.data(Qt::DisplayRole).toString();
     }
     return result;
}

void ObjectName::add_name(QString str)
{
     str = str.trimmed();
     
     QModelIndexList indexList = model->match(model->index(0, 0), Qt::DisplayRole, str);
     QModelIndex index;

     // If it doesn't exist yet
     if (indexList.empty()) {
          // Add it
          list_.append(str);
          model->setStringList(list_);          
     }

     // Select the newly created name
     this->set_selected_name(str);
}

void ObjectName::set_selected_name(QString str)
{
     QModelIndexList indexList = model->match(model->index(0, 0), Qt::DisplayRole, str);
     QModelIndex index;
     if (!indexList.empty()) {
          index = indexList.first();
     } else {
          // Add it
          list_.append(str);
          model->setStringList(list_);          
          
          // Find it (again)
          indexList = model->match(model->index(0, 0), Qt::DisplayRole, str);
          if (!indexList.empty()) {
               index = indexList.first();
          }
     }

     // Select the item in the list
     ui.listView->selectionModel()->select(index, QItemSelectionModel::SelectCurrent);
}

void ObjectName::delete_name()
{
     QModelIndexList list = ui.listView->selectionModel()->selectedIndexes();     
     foreach(const QModelIndex &index, list){          
          list_.removeOne(index.data(Qt::DisplayRole).toString());
     }
     model->setStringList(list_);
}

void ObjectName::add_name()
{     
     if (!(ui.text_name->text().trimmed().isEmpty())) {
          this->add_name(ui.text_name->text());
     }
     ui.text_name->clear();
}

void ObjectName::writeSettings()
{
     //QSettings settings;
     //// Window size and position
     //settings.beginGroup("object_name_asdf");
     //settings.setValue("size", size());
     //settings.setValue("pos", pos());               
     //settings.endGroup();
}

void ObjectName::readSettings()
{
     //QSettings settings;
     //settings.beginGroup("object_name_asdf");
     //
     //// Window size
     //resize(settings.value("size", QSize(400, 400)).toSize());
     //move(settings.value("pos", QPoint(200, 200)).toPoint());
     //     
     //settings.endGroup();
}


// Iterate over QList
//QList<QString>::iterator i;
//for (i = list_.begin(); i != list_.end(); ++i) {
//     cout << i->toStdString() << endl;
//}

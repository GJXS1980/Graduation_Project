# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel


class SettingsNameItem(QStandardItem):

    ITEM_TYPE = QStandardItem.UserType + 80

    def __init__(self, name, tooltip=''):
        QStandardItem.__init__(self, name)
        self.name = name
        self.tooltip = tooltip

    def type(self):
        return SettingsNameItem.ITEM_TYPE

    def data(self, role):
        '''
        The view asks us for all sorts of information about our data...
        @param role: the art of the data
        @type role: U{QtCore.Qt.DisplayRole<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if role == Qt.DisplayRole:
            # return the displayed item name
            return self.name
        elif role == Qt.ToolTipRole:
            # return the tooltip of the item
            return self.tooltip
        else:
            # We don't care about anything else, so return None
            return QStandardItem.data(self, role)


class SettingsValueItem(QStandardItem):

    ITEM_TYPE = QStandardItem.UserType + 81

    EDIT_TYPE_AUTODETECT = 0
    EDIT_TYPE_FOLDER = 1
    EDIT_TYPE_LIST = 2

    def __init__(self, value, (settings, attrname)=(None, None),
                 edit_type=0,
                 value_default=None, value_min=None, value_max=None, value_list=[], value_step=None):
        '''
        :param value: the current value
        :type value: any std types
        :param settings: the object, which contains `attrname` as property and
        provide the parameter changes
        :type settings: object (Settings)
        :param attrname: the parameter name, which is available as property in
        `settings` object.
        :type attrname: str
        :param edit_type: the editor type will be detected automatically by default.
        For different editors you can set manually the `EDIT_TYPE_*`
        :type edit_type: int (`EDIT_TYPE_*`)
        :param value_default: the default value, is needed for reset functionality
        :param value_min: the maximum value (used by int or float)
        :param value_max: the minimum value (used by int or float)
        :param value_list: the list of values used for comboboxes
        '''
        QStandardItem.__init__(self, '%s' % value)
        self._attrname = attrname
        self._value = value
        self._value_default = value_default
        self._value_min = value_min
        self._value_max = value_max
        self._settings = settings
        self._edit_type = edit_type
        self._value_list = value_list
        self._value_step = value_step

    def type(self):
        return SettingsValueItem.ITEM_TYPE

    def attrname(self):
        return self._attrname

    def value(self):
        return self._value

    def value_default(self):
        return self._value_default

    def value_min(self):
        return self._value_min

    def value_max(self):
        return self._value_max

    def value_step(self):
        return self._value_step

    def edit_type(self):
        return self._edit_type

    def value_list(self):
        return self._value_list

    def data(self, role):
        '''
        The view asks us for all sorts of information about our data...
        @param role: the art of the data
        @type role: U{QtCore.Qt.DisplayRole<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if role == Qt.DisplayRole:
            # return the displayed item name
            return '%s' % self._value
#     elif role == Qt.DecorationRole:
#       pass
        elif role == Qt.EditRole:
            return self._value
        else:
            # We don't care about anything else, so return None
            return QStandardItem.data(self, role)

    def setData(self, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            self._value = value
            if hasattr(self._settings, self._attrname):
                setattr(self._settings, self._attrname, value)
        return QStandardItem.setData(self, value, role)


class SettingsGroupItem(QStandardItem):

    ITEM_TYPE = QStandardItem.UserType + 82

    def __init__(self, name):
        QStandardItem.__init__(self, name)
        self.name = name

    def type(self):
        return SettingsGroupItem.ITEM_TYPE

    def data(self, role):
        '''
        The view asks us for all sorts of information about our data...
        @param role: the art of the data
        @type role: U{QtCore.Qt.DisplayRole<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if role == Qt.DisplayRole:
            # return the displayed item name
            return self.name
#     elif role == Qt.DecorationRole:
#       pass
        else:
            # We don't care about anything else, so return None
            return QStandardItem.data(self, role)

    @classmethod
    def getGroupItemList(self, name):
        '''
        Creates the list of the items . This list is used for the
        visualization of settings group data as a table row.
        @param name: the group name
        @type name: C{str}
        @rtype: C{[L{SettingsGroupItem} and U{QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}]}
        '''
        items = []
        item = SettingsGroupItem(name)
        items.append(item)
#     item = QStandardItem('')
#     items.append(item)
        return items

    @classmethod
    def getSettingsItemList(self, name, value, (settings, attrname)=(None, None),
                            tooltip='', edit_type=SettingsValueItem.EDIT_TYPE_AUTODETECT,
                            value_default=None, value_min=None, value_max=None, value_list=[], value_step=None):
        '''
        Creates the list of the items . This list is used for the
        visualization of settings group data as a table row.
        For paramters see `SettingsValueItem()`
        @rtype: C{[L{SettingsGroupItem} and U{QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}]}
        '''
        items = []
        item = SettingsNameItem(name, tooltip)
        items.append(item)
        item = SettingsValueItem(value, (settings, attrname), edit_type,
                                 value_default, value_min, value_max, value_list, value_step)
        items.append(item)
        return items


class SettingsModel(QStandardItemModel):
    '''
    The model to manage the settings.
    '''
    header = [('Parameter', 160), ('Value', -1)]
    '''@ivar: the list with columns C{[(name, width), ...]}'''

    def __init__(self):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self)
        self.setColumnCount(len(SettingsModel.header))
        self.setHorizontalHeaderLabels([label for label, _ in SettingsModel.header])
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              Overloaded methods                    %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def flags(self, index):
        '''
        @param index: parent of the list
        @type index: U{QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}
        @return: Flag or the requested item
        @rtype: U{QtCore.Qt.ItemFlag<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if not index.isValid():
            return Qt.NoItemFlags
        try:
            item = self.itemFromIndex(index)
            result = Qt.ItemIsSelectable | Qt.ItemIsEnabled
            if item.type() in [SettingsValueItem.ITEM_TYPE]:
                result = result | Qt.ItemIsEditable
            return result
        except:
            import traceback
            print traceback.format_exc(1)
            return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              External usage                        %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def init_settings(self, settings):
        '''
        Updates the model data.
        @param settings: a dictionary with settings name and values.
        @type settings: C{dict(str: parameter of L{SettingsGroupItem.getSettingsItemList()}, ...)}
        '''
        # remove all current items
        root = self.invisibleRootItem()
        while root.rowCount():
            root.removeRow(0)
        self.pyqt_workaround.clear()
        # add new items
        try:
            for name, value in settings.items():
                self._add_item(root, name, value)
        except:
            import traceback
            print traceback.format_exc(1)

    def _add_item(self, root, name, value):
        if isinstance(value, dict):
            new_item_row = SettingsGroupItem.getGroupItemList(name)
            root.appendRow(new_item_row)
            self.pyqt_workaround['group_%s' % name] = new_item_row[0]
            for name, value in value.items():
                self._add_item(new_item_row[0], name, value)
        else:
            args = (name,
                    self._get_settings_param(value, 'value'),
                    (self._get_settings_param(value, 'settings'), self._get_settings_param(value, 'attrname')),
                    self._get_settings_param(value, 'tooltip', ''),
                    self._get_settings_param(value, 'edit_type', SettingsValueItem.EDIT_TYPE_AUTODETECT),
                    self._get_settings_param(value, 'value_default'),
                    self._get_settings_param(value, 'value_min'),
                    self._get_settings_param(value, 'value_max'),
                    self._get_settings_param(value, 'value_list'),
                    self._get_settings_param(value, 'value_step')
                    )
            new_item_row = SettingsGroupItem.getSettingsItemList(*args)
            root.appendRow(new_item_row)
            self.pyqt_workaround[name] = new_item_row[0]

    def _get_settings_param(self, entry, param, default=None):
        try:
            return entry[0][param]
        except:
            return default

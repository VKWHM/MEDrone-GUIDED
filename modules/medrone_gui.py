# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'untitled.ui'
#
# Created by: PyQt5 UI code generator 5.14.0
#
# WARNING! All changes made in this file will be lost!


from array import *
import math
from PyQt5 import QtCore, QtGui, QtWidgets
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
from pymavlink import mavutil
import queue
import json
from medrone import GPSLocation


hedef_index = []
hedef_list = []


def Convert(lst):
    # print(f"lst {lst}")
    tmp_dict = {}
    for i in range(len(lst)):
        tmp_dict[str(i)] = LocationGlobalRelative(
            lst[i][0], lst[i][1], lst[i][2])
    # print(f"tmp {tmp_dict}")
    return tmp_dict


def mesafeHesapla(hedefKonum, mevcutKonum):
    dlat = hedefKonum.lat - mevcutKonum.lat
    dlon = hedefKonum.lon - mevcutKonum.lon
    return math.sqrt((dlon * dlon) + (dlat * dlat)) * 1.113195e5


class Ui_MEDrone_main_menu(object):
    def __init__(self, home, q):
        self.home = home
        self.q = q

    def setupUi(self, MEDrone_main_menu):
        MEDrone_main_menu.setObjectName("MEDrone_rota_detay")
        MEDrone_main_menu.setEnabled(True)
        MEDrone_main_menu.resize(702, 592)
        MEDrone_main_menu.setAutoFillBackground(False)
        self.e1_box = QtWidgets.QGroupBox(MEDrone_main_menu)
        self.e1_box.setGeometry(QtCore.QRect(30, 20, 500, 101))
        self.e1_box.setStyleSheet("")
        self.e1_box.setObjectName("e1_box")
        self.e1_box_lat = QtWidgets.QLabel(self.e1_box)
        self.e1_box_lat.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.e1_box_lat.setObjectName("e1_box_lat")
        self.e1_box_lat_val = QtWidgets.QLabel(self.e1_box)
        self.e1_box_lat_val.setGeometry(QtCore.QRect(80, 30, 130, 17))
        self.e1_box_lat_val.setObjectName("e1_box_lat_val")
        self.e1_box_long_val = QtWidgets.QLabel(self.e1_box)
        self.e1_box_long_val.setGeometry(QtCore.QRect(260, 30, 130, 17))
        self.e1_box_long_val.setObjectName("e1_box_long_val")
        self.e1_box_long = QtWidgets.QLabel(self.e1_box)
        self.e1_box_long.setGeometry(QtCore.QRect(200, 30, 67, 17))
        self.e1_box_long.setObjectName("e1_box_long")
        self.e1_box_ilac = QtWidgets.QLabel(self.e1_box)
        self.e1_box_ilac.setGeometry(QtCore.QRect(10, 70, 67, 17))
        self.e1_box_ilac.setObjectName("e1_box_ilac")
        self.e1_box_status = QtWidgets.QLabel(self.e1_box)
        self.e1_box_status.setGeometry(QtCore.QRect(200, 70, 67, 17))
        self.e1_box_status.setObjectName("e1_box_status")
        self.e1_box_status_val = QtWidgets.QLabel(self.e1_box)
        self.e1_box_status_val.setGeometry(QtCore.QRect(260, 70, 67, 17))
        self.e1_box_status_val.setObjectName("e1_box_status_val")
        self.e1_box_ilac_val = QtWidgets.QLabel(self.e1_box)
        self.e1_box_ilac_val.setGeometry(QtCore.QRect(80, 70, 67, 17))
        self.e1_box_ilac_val.setObjectName("e1_box_ilac_val")
        self.groupBox_2 = QtWidgets.QGroupBox(MEDrone_main_menu)
        self.groupBox_2.setGeometry(QtCore.QRect(30, 130, 500, 101))
        self.groupBox_2.setStyleSheet("")
        self.groupBox_2.setObjectName("groupBox_2")
        self.label_8 = QtWidgets.QLabel(self.groupBox_2)
        self.label_8.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.groupBox_2)
        self.label_9.setGeometry(QtCore.QRect(80, 30, 130, 17))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.groupBox_2)
        self.label_10.setGeometry(QtCore.QRect(260, 30, 130, 17))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.groupBox_2)
        self.label_11.setGeometry(QtCore.QRect(200, 30, 67, 17))
        self.label_11.setObjectName("label_11")
        self.label_12 = QtWidgets.QLabel(self.groupBox_2)
        self.label_12.setGeometry(QtCore.QRect(10, 70, 67, 17))
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.groupBox_2)
        self.label_13.setGeometry(QtCore.QRect(200, 70, 67, 17))
        self.label_13.setObjectName("label_13")
        self.label_14 = QtWidgets.QLabel(self.groupBox_2)
        self.label_14.setGeometry(QtCore.QRect(260, 70, 67, 17))
        self.label_14.setObjectName("label_14")
        self.label_39 = QtWidgets.QLabel(self.groupBox_2)
        self.label_39.setGeometry(QtCore.QRect(100, 100, 67, 17))
        self.label_39.setObjectName("label_39")
        self.label_40 = QtWidgets.QLabel(self.groupBox_2)
        self.label_40.setGeometry(QtCore.QRect(80, 70, 67, 17))
        self.label_40.setObjectName("label_40")
        self.groupBox_3 = QtWidgets.QGroupBox(MEDrone_main_menu)
        self.groupBox_3.setGeometry(QtCore.QRect(30, 240, 500, 101))
        self.groupBox_3.setStyleSheet("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.label_15 = QtWidgets.QLabel(self.groupBox_3)
        self.label_15.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(self.groupBox_3)
        self.label_16.setGeometry(QtCore.QRect(80, 30, 130, 17))
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.groupBox_3)
        self.label_17.setGeometry(QtCore.QRect(260, 30, 130, 17))
        self.label_17.setObjectName("label_17")
        self.label_18 = QtWidgets.QLabel(self.groupBox_3)
        self.label_18.setGeometry(QtCore.QRect(200, 30, 67, 17))
        self.label_18.setObjectName("label_18")
        self.label_19 = QtWidgets.QLabel(self.groupBox_3)
        self.label_19.setGeometry(QtCore.QRect(10, 70, 67, 17))
        self.label_19.setObjectName("label_19")
        self.label_20 = QtWidgets.QLabel(self.groupBox_3)
        self.label_20.setGeometry(QtCore.QRect(200, 70, 67, 17))
        self.label_20.setObjectName("label_20")
        self.label_21 = QtWidgets.QLabel(self.groupBox_3)
        self.label_21.setGeometry(QtCore.QRect(260, 70, 67, 17))
        self.label_21.setObjectName("label_21")
        self.label_38 = QtWidgets.QLabel(self.groupBox_3)
        self.label_38.setGeometry(QtCore.QRect(80, 70, 67, 17))
        self.label_38.setObjectName("label_38")
        self.pushButton = QtWidgets.QPushButton(MEDrone_main_menu)
        self.pushButton.setGeometry(QtCore.QRect(558, 30, 121, 25))
        self.pushButton.setObjectName("pushButton")
        self.pushButton.raise_()
        self.e1_box.raise_()
        self.groupBox_2.raise_()
        self.groupBox_3.raise_()
        self.pushButton.raise_()

        self.retranslateUi(MEDrone_main_menu)
        QtCore.QMetaObject.connectSlotsByName(MEDrone_main_menu)

    def menu(self):
        gorev = Ui_MEDrone_siralama(self.home, self.q)
        self.window = QtWidgets.QDialog()
        self.ui = gorev
        self.ui.setupUi(self.window)
        self.window.show()

    def retranslateUi(self, MEDrone_main_menu):
        # print(hedef_index)
        _translate = QtCore.QCoreApplication.translate
        MEDrone_main_menu.setWindowTitle(
            _translate("MEDrone_rota_detay", "Medrone Ana Menü")
        )
        self.e1_box.setTitle(_translate("MEDrone_rota_detay", "Eczane 1"))
        self.e1_box_lat.setText(_translate("MEDrone_rota_detay", "Enlem:"))
        if hedef_index[0] == 1:
            self.e1_box_lat_val.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][0]:.3f}")))
            self.e1_box_long_val.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][1]:.3f}")))
        else:
            self.e1_box_lat_val.setText(
                _translate("MEDrone_rota_detay", "..."))
            self.e1_box_long_val.setText(
                _translate("MEDrone_rota_detay", "..."))
        self.e1_box_long.setText(_translate("MEDrone_rota_detay", "Boylam"))
        self.e1_box_ilac.setText(_translate("MEDrone_rota_detay", "İlaç:"))
        self.e1_box_status.setText(_translate("MEDrone_rota_detay", "Durum"))
        self.e1_box_status_val.setText(_translate("MEDrone_rota_detay", "..."))
        self.e1_box_ilac_val.setText(_translate("MEDrone_rota_detay", "..."))
        self.groupBox_2.setTitle(_translate("MEDrone_rota_detay", "Eczane 2"))
        self.label_8.setText(_translate("MEDrone_rota_detay", "Enlem:"))
        if hedef_index[0] == 2:
            self.label_9.setText(
                _translate("MEDrone_rota_detay", str(f"{hedef_list[0][0]:.3f}")))
            self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][1]:.3f}"))
            )
        # 2
        elif len(hedef_list) == 4:
            if hedef_index[0] == 2:
                self.label_9.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][0]:.3f}"))
                )
                self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][1]:.3f}"))
                )
            elif hedef_index[1] == 2:
                self.label_9.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][0]:.3f}"))
                )
                self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][1]:.3f}"))
                )

        elif len(hedef_list) == 3:
            if hedef_index[0] == 2:
                self.label_9.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][0]:.3f}"))
                )
                self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][1]:.3f}"))
                )
            elif hedef_index[1] == 2:
                self.label_9.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][0]:.3f}"))
                )
                self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][1]:.3f}"))
                )

        elif len(hedef_list) == 2:
            if hedef_index[1] == 2:
                self.label_9.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][0]:.3f}"))
                )
                self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][1]:.3f}"))
                )

        elif len(hedef_list) == 5:
            self.label_9.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][0]:.3f}"))
            )
            self.label_10.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][1]:.3f}"))
            )
        else:
            self.label_9.setText(_translate("MEDrone_rota_detay", "..."))
            self.label_10.setText(_translate("MEDrone_rota_detay", "..."))
        self.label_11.setText(_translate("MEDrone_rota_detay", "Boylam"))
        self.label_12.setText(_translate("MEDrone_rota_detay", "İlaç:"))
        self.label_13.setText(_translate("MEDrone_rota_detay", "Durum"))
        self.label_14.setText(_translate("MEDrone_rota_detay", "..."))
        self.label_39.setText(_translate("MEDrone_rota_detay", "..."))
        self.label_40.setText(_translate("MEDrone_rota_detay", "..."))
        self.groupBox_3.setTitle(_translate("MEDrone_rota_detay", "Eczane 3"))
        self.label_15.setText(_translate("MEDrone_rota_detay", "Enlem:"))
        if hedef_index[0] == 3:
            self.label_16.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][0]:.3f}"))
            )
            self.label_17.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[0][1]:.3f}"))
            )

        elif len(hedef_list) == 2:
            if hedef_index[1] == 3:
                self.label_16.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[1][0]:.3f}"))
                )
                self.label_17.setText(_translate("MEDrone_rota_detay",str(f"{hedef_list[1][1]:.3f}"))
                )

        elif len(hedef_list) == 3:
            self.label_16.setText( _translate("MEDrone_rota_detay", str(f"{hedef_list[2][0]:.3f}"))
            )
            self.label_17.setText(_translate("MEDrone_rota_detay", str(f"{hedef_list[2][1]:.3f}"))
            )
        else:
            self.label_16.setText(_translate("MEDrone_rota_detay", "..."))
            self.label_17.setText(_translate("MEDrone_rota_detay", "..."))
        self.label_18.setText(_translate("MEDrone_rota_detay", "Boylam"))
        self.label_19.setText(_translate("MEDrone_rota_detay", "İlaç :"))
        self.label_20.setText(_translate("MEDrone_rota_detay", "Durum"))
        self.label_21.setText(_translate("MEDrone_rota_detay", "..."))
        self.label_38.setText(_translate("MEDrone_rota_detay", "..."))
        self.pushButton.setText(_translate(
            "MEDrone_rota_detay", "Görev Oluştur"))
        self.pushButton.clicked.connect(self.menu)


class Ui_MEDrone_siralama(object):
    def __init__(self, home, q):
        self.home = home
        self.q = q

    def setupUi(self, MEDrone_siralama):
        global hedef_sira
        hedef_sira = []
        MEDrone_siralama.setObjectName("MEDrone_siralama")
        MEDrone_siralama.resize(420, 300)
        self.pushButton = QtWidgets.QPushButton(MEDrone_siralama)
        self.pushButton.setGeometry(QtCore.QRect(130, 240, 151, 25))
        self.pushButton.setObjectName("pushButton")

        # E3
        self.e3 = QtWidgets.QSplitter(MEDrone_siralama)
        self.e3.setGeometry(QtCore.QRect(80, 110, 241, 25))
        self.e3.setOrientation(QtCore.Qt.Horizontal)
        self.e3.setObjectName("e3")
        self.label_3 = QtWidgets.QLabel(self.e3)
        self.label_3.setObjectName("label_3")
        self.e3_acil = QtWidgets.QCheckBox(self.e3)
        self.e3_acil.setObjectName("e3_acil")
        self.e3_type = QtWidgets.QComboBox(self.e3)
        self.e3_type.setObjectName("e3_type")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        self.e3_type.addItem("")
        # E2
        self.e2 = QtWidgets.QSplitter(MEDrone_siralama)
        self.e2.setGeometry(QtCore.QRect(80, 70, 241, 25))
        self.e2.setOrientation(QtCore.Qt.Horizontal)
        self.e2.setObjectName("e2")
        self.label_2 = QtWidgets.QLabel(self.e2)
        self.label_2.setObjectName("label_2")
        self.e2_acil = QtWidgets.QCheckBox(self.e2)
        self.e2_acil.setObjectName("e2_acil")
        self.e2_type = QtWidgets.QComboBox(self.e2)
        self.e2_type.setObjectName("e2_type")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        self.e2_type.addItem("")
        # E1
        self.e1 = QtWidgets.QSplitter(MEDrone_siralama)
        self.e1.setGeometry(QtCore.QRect(80, 30, 241, 25))
        self.e1.setOrientation(QtCore.Qt.Horizontal)
        self.e1.setObjectName("e1")
        self.label = QtWidgets.QLabel(self.e1)
        self.label.setObjectName("label")
        self.e1_acil = QtWidgets.QCheckBox(self.e1)
        self.e1_acil.setObjectName("e1_acil")
        self.e1_type = QtWidgets.QComboBox(self.e1)
        self.e1_type.setObjectName("e1_type")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.e1_type.addItem("")
        self.retranslateUi(MEDrone_siralama)
        QtCore.QMetaObject.connectSlotsByName(MEDrone_siralama)
        self.e1.hide()
        self.e2.hide()
        self.e3.hide()
        for i in range(len(hedef_index)):
            if hedef_index[i] == 1:
                self.e1.show()
            if hedef_index[i] == 2:
                if len(hedef_index) == 1:
                    self.e2.setGeometry(QtCore.QRect(80, 30, 241, 25))
                if len(hedef_index) == 2:
                    if hedef_index[0] == 2:
                        self.e2.setGeometry(QtCore.QRect(80, 30, 241, 25))
                    else:
                        self.e2.setGeometry(QtCore.QRect(80, 70, 241, 25))
                self.e2.show()
            if hedef_index[i] == 3:
                if len(hedef_index) == 1:
                    self.e3.setGeometry(QtCore.QRect(80, 30, 241, 25))
                elif len(hedef_index) == 2:
                    if hedef_index[0] == 3:
                        self.e3.setGeometry(QtCore.QRect(80, 30, 241, 25))
                    elif hedef_index[1] == 3:
                        self.e3.setGeometry(QtCore.QRect(80, 70, 241, 25))
                elif len(hedef_index) == 3:
                    if hedef_index[0] == 3:
                        self.e3.setGeometry(QtCore.QRect(80, 30, 241, 25))
                    elif hedef_index[1] == 3:
                        self.e3.setGeometry(QtCore.QRect(80, 70, 241, 25))
                    elif hedef_index[2] == 3:
                        self.e3.setGeometry(QtCore.QRect(80, 110, 241, 25))
                elif len(hedef_index) == 4:
                    if hedef_index[0] == 3:
                        self.e4.setGeometry(QtCore.QRect(80, 30, 241, 25))
                    elif hedef_index[1] == 3:
                        self.e4.setGeometry(QtCore.QRect(80, 70, 241, 25))
                    elif hedef_index[2] == 3:
                        self.e4.setGeometry(QtCore.QRect(80, 110, 241, 25))
                self.e3.show()

    acil_true = [0, 0, 0, 0, 0]

    def sirala(self):
        global konumlar, mesafeler, gorev_sira, main_gorev_sira
        main_gorev_sira = {}
        gorev_sira = {}
        konumlar = {}
        mesafeler = {}
        acl_olm_say_sira_index = []
        for i in range(len(hedef_list)):
            konumlar["home"] = self.home
            konumlar[str(i + 1)] = LocationGlobalRelative(
                hedef_list[i][0], hedef_list[i][1], hedef_list[i][2]
            )
            ecz_ve_home_mesafe = mesafeHesapla(
                konumlar[str(i + 1)], konumlar["home"])
            hedef_sira[i][5] = ecz_ve_home_mesafe
        # print(mesafeler)
        # print(konumlar)

        # Aciliyet sıralama
        siralama = hedef_sira.copy()
        siralama.sort(key=lambda siralama: siralama[3], reverse=True)
        # print(siralama)

        # aciliyet filtreleme
        acil_liste = list(filter(lambda x: (x[3] == 1), siralama))
        acil_liste.sort(key=lambda siralama: siralama[5])
        # print("Acil Eczaneler:\n")
        # print(acil_liste)
        acil_list_dict = Convert(acil_liste)
        # print(acil_list_dict)

        # print("Acil Olmayan Eczaneler:\n")
        # Geriye kalan eczaneleri yani 0 ları filtreledik
        acil_olmayan_liste = list(filter(lambda y: (y[3] == 0), siralama))
        acil_olmayan_liste.sort(key=lambda siralama: siralama[5])
        # print(acil_olmayan_liste)

        acil_olm_list_dict = Convert(acil_olmayan_liste)
        # print(acil_olm_list_dict)

        acil_sayi = len(acil_liste)
        acil_olm_sayi = len(acil_olmayan_liste)

        if acil_sayi > 0:
            for i in range(acil_sayi):
                gorev_sira[i] = LocationGlobalRelative(
                    acil_liste[(i)][0], acil_liste[(i)][1], acil_liste[(i)][2]
                )
                main_gorev_sira[i] = [
                    acil_liste[(i)][0],
                    acil_liste[(i)][1],
                    acil_liste[(i)][2],
                ]
        # print(gorev_sira)
        if acil_olm_sayi == 3:
            acl_olm_say_sira2 = []
            acl_olm_say_sira_index2 = []
            gorev_sira[0] = LocationGlobalRelative(
                acil_olmayan_liste[0][0],
                acil_olmayan_liste[0][1],
                acil_olmayan_liste[0][2],
            )
            main_gorev_sira[0] = [
                acil_olmayan_liste[0][0],
                acil_olmayan_liste[0][1],
                acil_olmayan_liste[0][2],
            ]
            for i in range(acil_olm_sayi):
                if i > 0:
                    # print(gorev_sira)
                    acl_olm_say_sira2.append(
                        mesafeHesapla(
                            acil_olm_list_dict[str(i)], gorev_sira[0])
                    )
                    acl_olm_say_sira_index2.append(i + 1)
                # print(acl_olm_say_sira2)
                acl_olm_say_yedek = acl_olm_say_sira2.copy()
                for i in range(acil_olm_sayi):
                    acl_olm_say_sira2.sort(key=lambda x: acl_olm_say_sira2)
                if acl_olm_say_sira2 == acl_olm_say_yedek:
                    for i in range(acil_olm_sayi):
                        if i > 0:
                            main_gorev_sira[i] = [
                                acil_olmayan_liste[i][0],
                                acil_olmayan_liste[i][1],
                                acil_olmayan_liste[i][2],
                            ]
                else:
                    main_gorev_sira[1] = [
                        acil_olmayan_liste[2][0],
                        acil_olmayan_liste[2][1],
                        acil_olmayan_liste[2][2],
                    ]
                    main_gorev_sira[2] = [
                        acil_olmayan_liste[1][0],
                        acil_olmayan_liste[1][1],
                        acil_olmayan_liste[1][2],
                    ]
        elif acil_olm_sayi == 2:
            if len(acil_liste) == 0:
                for i in range(acil_sayi):
                    gorev_sira[i] = acil_olm_list_dict[str(i)]
                    main_gorev_sira[i] = [
                        acil_olmayan_liste[(i)][0],
                        acil_olmayan_liste[(i)][1],
                        acil_olmayan_liste[(i)][2],
                    ]
            else:
                acl_olm_say_sira = []
                # print("olm list")
                # print(acil_olm_list_dict)
                for i in range(acil_olm_sayi):
                    # acil noktasından sonra geriye kalan noktalar ile acil noktası arasındaki
                    # mesafeler hesaplanıp küçükten büyüğe sıraladık
                    # amacımız en kısa yol algoritmasına göre görevi tamamlamak
                    acl_olm_say_sira.append(
                        mesafeHesapla(
                            acil_olm_list_dict[str(i)], gorev_sira[0])
                    )
                    acl_olm_say_sira_index.append(i)
                # print(acl_olm_say_sira)
                acl_olm_say_yedek = acl_olm_say_sira.copy()
                for i in range(acil_olm_sayi):
                    acl_olm_say_sira.sort(key=lambda x: acl_olm_say_sira)
                if acl_olm_say_sira == acl_olm_say_yedek:
                    for i in range(acil_olm_sayi):
                        main_gorev_sira[i + 1] = [
                            acil_olmayan_liste[i][0],
                            acil_olmayan_liste[i][1],
                            acil_olmayan_liste[i][2],
                        ]
                else:
                    main_gorev_sira[1] = [
                        acil_olmayan_liste[1][0],
                        acil_olmayan_liste[1][1],
                        acil_olmayan_liste[1][2],
                    ]
                    main_gorev_sira[2] = [
                        acil_olmayan_liste[0][0],
                        acil_olmayan_liste[0][1],
                        acil_olmayan_liste[0][2],
                    ]
        elif acil_olm_sayi == 1:
            if acil_sayi == 0:
                main_gorev_sira[0] = [
                    acil_olmayan_liste[0][0],
                    acil_olmayan_liste[0][1],
                    acil_olmayan_liste[0][2],
                ]
            if acil_sayi == 1:
                main_gorev_sira[1] = [
                    acil_olmayan_liste[0][0],
                    acil_olmayan_liste[0][1],
                    acil_olmayan_liste[0][2],
                ]
            if acil_sayi == 2:
                main_gorev_sira[2] = [
                    acil_olmayan_liste[0][0],
                    acil_olmayan_liste[0][1],
                    acil_olmayan_liste[0][2],
                ]

        # print(gorev_sira)
        # print(main_gorev_sira)

        ecz1_acilmi = hedef_sira[0][3]
        ecz2_acilmi = hedef_sira[1][3]
        ecz3_acilmi = hedef_sira[2][3]
        returnOfSP = self.shortest_path(ecz1_acilmi, ecz2_acilmi, ecz3_acilmi)
        global eczaneListOrdered
        eczaneListOrdered = []
        possiblePathList = [
            [1, 2, 3],
            [1, 3, 2],
            [2, 1, 3],
            [2, 3, 1],
            [3, 1, 2],
            [3, 2, 1],
        ]
        if possiblePathList[0] == returnOfSP[0]:
            eczaneListOrdered.append(hedef_list[0])
            eczaneListOrdered.append(hedef_list[1])
            eczaneListOrdered.append(hedef_list[2])
        elif possiblePathList[1] == returnOfSP[0]:
            eczaneListOrdered.append(hedef_list[0])
            eczaneListOrdered.append(hedef_list[2])
            eczaneListOrdered.append(hedef_list[1])
        elif possiblePathList[2] == returnOfSP[0]:
            eczaneListOrdered.append(hedef_list[1])
            eczaneListOrdered.append(hedef_list[0])
            eczaneListOrdered.append(hedef_list[2])
        elif possiblePathList[3] == returnOfSP[0]:
            eczaneListOrdered.append(hedef_list[1])
            eczaneListOrdered.append(hedef_list[2])
            eczaneListOrdered.append(hedef_list[0])
        elif possiblePathList[4] == returnOfSP[0]:
            eczaneListOrdered.append(hedef_list[2])
            eczaneListOrdered.append(hedef_list[0])
            eczaneListOrdered.append(hedef_list[1])
        elif possiblePathList[5] == returnOfSP[0]:
            eczaneListOrdered.append(hedef_list[2])
            eczaneListOrdered.append(hedef_list[1])
            eczaneListOrdered.append(hedef_list[0])

        # print(returnOfSP)
        print(acil_liste)
         
        print(acil_olmayan_liste)
        servoIds = []
        for i in returnOfSP[0]:
            servoIds.append(i+8)
        acil_dict = [{'coordinates': degisken[:3], 'urgency': 1}
                 for degisken in acil_liste]
        acilsiz_dict = [{'coordinates': degisken[:3], 'urgency': 0}
                 for degisken in acil_olmayan_liste]
        acil_dict.extend(acilsiz_dict)
        finalList = zip(acil_dict, servoIds)
        _list = []
        for degisken in finalList:
            degisken[0].update({'servoId':degisken[1]}) 
            _list.append(degisken[0])

        with open('src/wp.json', 'w') as f:
            json.dump({'sort_status': 0, 'points': _list}, f, indent=4)
        app.quit()

    def shortest_path(self, ecz1_acil, ecz2_acil, ecz3_acil):

        # 1, 2, 3 gidilecek eczaneler olsun (parametrik).
        # Gidilebilecek yollar aşağıdaki gibi olmak zorundadır, 6 ihtimal var.
        # Home ile tüm eczaneler arası mesafeyi belirle.

        wp_home = self.home
        wp_ecz1 = LocationGlobalRelative(
            hedef_list[0][0], hedef_list[0][1], hedef_list[0][2]
        )
        wp_ecz2 = LocationGlobalRelative(
            hedef_list[1][0], hedef_list[1][1], hedef_list[1][2]
        )
        wp_ecz3 = LocationGlobalRelative(
            hedef_list[2][0], hedef_list[2][1], hedef_list[2][2]
        )
        distHomeEcz1 = mesafeHesapla(wp_ecz1, wp_home)
        distHomeEcz2 = mesafeHesapla(wp_ecz2, wp_home)
        distHomeEcz3 = mesafeHesapla(wp_ecz3, wp_home)
        distEcz1Ecz2 = mesafeHesapla(wp_ecz1, wp_ecz2)
        distEcz1Ecz3 = mesafeHesapla(wp_ecz1, wp_ecz3)
        distEcz2Ecz3 = mesafeHesapla(wp_ecz2, wp_ecz3)

        # print(wp_home, wp_ecz1, wp_ecz2, wp_ecz3)
        # print("-"*60)
        # print("DH1: "+str(distHomeEcz1) +" DH2: "+ str(distHomeEcz2) + " DH3: "+ str(distHomeEcz3))
        # print("D12: "+str(distEcz1Ecz2) +" D13: "+ str(distEcz1Ecz3) + " D23: "+ str(distEcz2Ecz3))

        possiblePathList = [
            [1, 2, 3],
            [1, 3, 2],
            [2, 1, 3],
            [2, 3, 1],
            [3, 1, 2],
            [3, 2, 1],
        ]  # Gidilebilecek yolların olası listesi
        # Örneğin [1, 2, 3] şu demek: Home'dan 1. eczaneye, 1.'den 2. eczaneye, 2.den 3. eczaneye, 3.'den tekrar home'a. (parametrik)

        sum1 = [
            distHomeEcz1 + distEcz1Ecz2 + distEcz2Ecz3 + distHomeEcz3,
            possiblePathList[0],
        ]  # Home -> 1 -> 2 -> 3 -> Home
        sum2 = [
            distHomeEcz1 + distEcz1Ecz3 + distEcz2Ecz3 + distHomeEcz2,
            possiblePathList[1],
        ]  # Home -> 1 -> 3 -> 2 -> Home
        sum3 = [
            distHomeEcz2 + distEcz1Ecz2 + distEcz1Ecz3 + distHomeEcz3,
            possiblePathList[2],
        ]  # Home -> 2 -> 1 -> 3 -> Home
        sum4 = [
            distHomeEcz2 + distEcz2Ecz3 + distEcz1Ecz3 + distHomeEcz1,
            possiblePathList[3],
        ]  # Home -> 2 -> 3 -> 1 -> Home
        sum5 = [
            distHomeEcz3 + distEcz1Ecz3 + distEcz1Ecz2 + distHomeEcz2,
            possiblePathList[4],
        ]  # Home -> 3 -> 1 -> 2 -> Home
        sum6 = [
            distHomeEcz3 + distEcz2Ecz3 + distEcz1Ecz2 + distHomeEcz1,
            possiblePathList[5],
        ]  # Home -> 3 -> 2 -> 1 -> Home

        # print(f"Yol1: {sum1}")
        # print(f"Yol2: {sum2}")
        # print(f"Yol3: {sum3}")
        # print(f"Yol4: {sum4}")
        # print(f"Yol5: {sum5}")
        # print(f"Yol6: {sum6}")

        # print(ecz1_acil, ecz2_acil, ecz3_acil)

        if ecz1_acil == True and ecz2_acil == False and ecz3_acil == False:
            if sum1[0] <= sum2[0]:
                return sum1[1], sum1[0]
            else:
                return sum2[1], sum2[0]
        elif ecz1_acil == False and ecz2_acil == True and ecz3_acil == False:
            if sum3[0] <= sum4[0]:
                return sum3[1], sum3[0]
            else:
                return sum4[1], sum4[0]
        elif ecz1_acil == False and ecz2_acil == False and ecz3_acil == True:
            if sum5[0] <= sum6[0]:
                return sum5[1], sum5[0]
            else:
                return sum6[1], sum6[0]
        elif ecz1_acil == True and ecz2_acil == True and ecz3_acil == False:
            if sum1[0] <= sum3[0]:
                return sum1[1], sum1[0]
            else:
                return sum3[1], sum3[0]
        elif ecz1_acil == True and ecz2_acil == False and ecz3_acil == True:
            if sum2[0] <= sum5[0]:
                return sum2[1], sum2[0]
            else:
                return sum5[1], sum5[0]
        elif ecz1_acil == False and ecz2_acil == True and ecz3_acil == True:
            if sum4[0] <= sum6[0]:
                return sum4[1], sum4[0]
            else:
                return sum6[1], sum6[0]
        else:
            minDist = min(sum1[0], sum2[0], sum3[0], sum4[0], sum5[0], sum6[0])
            # print(f"mindist {minDist}")
            if sum1[0] == minDist:
                return sum1[1], sum1[0]
            elif sum2[0] == minDist:
                return sum2[1], sum2[0]
            elif sum3[0] == minDist:
                return sum3[1], sum3[0]
            elif sum4[0] == minDist:
                return sum4[1], sum4[0]
            elif sum5[0] == minDist:
                return sum5[1], sum5[0]
            elif sum6[0] == minDist:
                return sum6[1], sum6[0]

    def crate_mission(self):
        global hedef_sira
        hedef_sira = []
        for i in range(len(hedef_index)):
            if hedef_index[i] == 1:
                if self.e1_acil.isChecked():
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            1,
                            self.e1_type.currentText(),
                            0,
                        ]
                    )
                else:
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            0,
                            self.e1_type.currentText(),
                            0,
                        ]
                    )

            elif hedef_index[i] == 2:
                if self.e2_acil.isChecked():
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            1,
                            self.e2_type.currentText(),
                            0,
                        ]
                    )
                else:
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            0,
                            self.e2_type.currentText(),
                            0,
                        ]
                    )

            elif hedef_index[i] == 3:
                if self.e3_acil.isChecked():
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            1,
                            self.e3_type.currentText(),
                            0,
                        ]
                    )
                else:
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            0,
                            self.e3_type.currentText(),
                            0,
                        ]
                    )

            elif hedef_index[i] == 4:
                if self.e4_acil.isChecked():
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            1,
                            self.e4_type.currentText(),
                            0,
                        ]
                    )
                else:
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            0,
                            self.e4_type.currentText(),
                            0,
                        ]
                    )

            elif hedef_index[i] == 5:
                if self.e5_acil.isChecked():
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            1,
                            self.e5_type.currentText(),
                            0,
                        ]
                    )
                else:
                    hedef_sira.append(
                        [
                            hedef_list[i][0],
                            hedef_list[i][1],
                            hedef_list[i][2],
                            0,
                            self.e5_type.currentText(),
                            0,
                        ]
                    )
        # print(hedef_sira)
        # print(hedef_index)

        self.sirala()

    def main_menu_veri_aktarma(self):
        pass

    def retranslateUi(self, MEDrone_siralama):
        _translate = QtCore.QCoreApplication.translate
        MEDrone_siralama.setWindowTitle(
            _translate("MEDrone_siralama", "MEDrone Hedef Sıralama")
        )
        self.pushButton.setText(_translate(
            "MEDrone_siralama", "Görev Rotası Oluştur"))
        self.pushButton.clicked.connect(self.crate_mission)
        self.label_3.setText(_translate("MEDrone_siralama", "Eczane 3"))
        self.e3_acil.setText(_translate("MEDrone_siralama", "Acil"))
        self.e3_type.setItemText(0, _translate("MEDrone_siralama", "İlaç A"))
        self.e3_type.setItemText(1, _translate("MEDrone_siralama", "İlaç B"))
        self.e3_type.setItemText(2, _translate("MEDrone_siralama", "İlaç C"))
        self.e3_type.setItemText(3, _translate("MEDrone_siralama", "İlaç D"))
        self.e3_type.setItemText(4, _translate("MEDrone_siralama", "İlaç E"))
        self.e3_type.setItemText(5, _translate("MEDrone_siralama", "İlaç F"))
        self.e3_type.setItemText(6, _translate("MEDrone_siralama", "İlaç G"))
        self.e3_type.setItemText(7, _translate("MEDrone_siralama", "İlaç H"))
        self.e3_type.setItemText(8, _translate("MEDrone_siralama", "İlaç I"))
        self.e3_type.setItemText(9, _translate("MEDrone_siralama", "İlaç J"))
        self.label_2.setText(_translate("MEDrone_siralama", "Eczane 2"))
        self.e2_acil.setText(_translate("MEDrone_siralama", "Acil"))
        self.e2_type.setItemText(0, _translate("MEDrone_siralama", "İlaç A"))
        self.e2_type.setItemText(1, _translate("MEDrone_siralama", "İlaç B"))
        self.e2_type.setItemText(2, _translate("MEDrone_siralama", "İlaç C"))
        self.e2_type.setItemText(3, _translate("MEDrone_siralama", "İlaç D"))
        self.e2_type.setItemText(4, _translate("MEDrone_siralama", "İlaç E"))
        self.e2_type.setItemText(5, _translate("MEDrone_siralama", "İlaç F"))
        self.e2_type.setItemText(6, _translate("MEDrone_siralama", "İlaç G"))
        self.e2_type.setItemText(7, _translate("MEDrone_siralama", "İlaç H"))
        self.e2_type.setItemText(8, _translate("MEDrone_siralama", "İlaç I"))
        self.e2_type.setItemText(9, _translate("MEDrone_siralama", "İlaç J"))
        self.label.setText(_translate("MEDrone_siralama", "Eczane 1"))
        self.e1_acil.setText(_translate("MEDrone_siralama", "Acil"))
        self.e1_type.setItemText(0, _translate("MEDrone_siralama", "İlaç A"))
        self.e1_type.setItemText(1, _translate("MEDrone_siralama", "İlaç B"))
        self.e1_type.setItemText(2, _translate("MEDrone_siralama", "İlaç C"))
        self.e1_type.setItemText(3, _translate("MEDrone_siralama", "İlaç D"))
        self.e1_type.setItemText(4, _translate("MEDrone_siralama", "İlaç E"))
        self.e1_type.setItemText(5, _translate("MEDrone_siralama", "İlaç F"))
        self.e1_type.setItemText(6, _translate("MEDrone_siralama", "İlaç G"))
        self.e1_type.setItemText(7, _translate("MEDrone_siralama", "İlaç H"))
        self.e1_type.setItemText(8, _translate("MEDrone_siralama", "İlaç I"))
        self.e1_type.setItemText(9, _translate("MEDrone_siralama", "İlaç J"))


class Ui_MEDrone(object):
    def __init__(self, home, q):
        self.home = home
        self.q = q

    def secondPage(self):
        gorev = Ui_MEDrone_main_menu(self.home, self.q)
        self.window = QtWidgets.QDialog()
        self.ui = gorev
        self.ui.setupUi(self.window)
        self.window.show()

    def setupUi(self, MEDrone):
        MEDrone.setObjectName("MEDrone")
        MEDrone.resize(547, 400)
        self.e1_lat = QtWidgets.QLineEdit(MEDrone)
        self.e1_lat.setGeometry(QtCore.QRect(100, 70, 151, 21))
        self.e1_lat.setObjectName("e1_lat")
        self.pushButton = QtWidgets.QPushButton(MEDrone)
        self.pushButton.setGeometry(QtCore.QRect(140, 310, 111, 21))
        self.pushButton.setObjectName("pushButton")
        self.label_2 = QtWidgets.QLabel(MEDrone)
        self.label_2.setGeometry(QtCore.QRect(10, 70, 61, 17))
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(MEDrone)
        self.label_3.setGeometry(QtCore.QRect(150, 30, 51, 17))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(MEDrone)
        self.label_4.setGeometry(QtCore.QRect(320, 30, 51, 17))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(MEDrone)
        self.label_5.setGeometry(QtCore.QRect(480, 30, 41, 17))
        self.label_5.setObjectName("label_5")
        self.e1_long = QtWidgets.QLineEdit(MEDrone)
        self.e1_long.setGeometry(QtCore.QRect(270, 70, 151, 21))
        self.e1_long.setObjectName("e1_long")
        self.e1_alt = QtWidgets.QLineEdit(MEDrone)
        self.e1_alt.setGeometry(QtCore.QRect(450, 70, 81, 21))
        self.e1_alt.setObjectName("e1_alt")
        self.pushButton_2 = QtWidgets.QPushButton(MEDrone)
        self.pushButton_2.setGeometry(QtCore.QRect(350, 310, 81, 21))
        self.pushButton_2.setObjectName("pushButton_2")
        self.e2_alt = QtWidgets.QLineEdit(MEDrone)
        self.e2_alt.setGeometry(QtCore.QRect(450, 110, 81, 21))
        self.e2_alt.setObjectName("e2_alt")
        self.e2_lat = QtWidgets.QLineEdit(MEDrone)
        self.e2_lat.setGeometry(QtCore.QRect(100, 110, 151, 21))
        self.e2_lat.setObjectName("e2_lat")
        self.e2_long = QtWidgets.QLineEdit(MEDrone)
        self.e2_long.setGeometry(QtCore.QRect(270, 110, 151, 21))
        self.e2_long.setObjectName("e2_long")
        self.label_6 = QtWidgets.QLabel(MEDrone)
        self.label_6.setGeometry(QtCore.QRect(10, 110, 61, 17))
        self.label_6.setObjectName("label_6")
        self.e3_alt = QtWidgets.QLineEdit(MEDrone)
        self.e3_alt.setGeometry(QtCore.QRect(450, 150, 81, 21))
        self.e3_alt.setObjectName("e3_alt")
        self.e3_lat = QtWidgets.QLineEdit(MEDrone)
        self.e3_lat.setGeometry(QtCore.QRect(100, 150, 151, 21))
        self.e3_lat.setObjectName("e3_lat")
        self.e3_long = QtWidgets.QLineEdit(MEDrone)
        self.e3_long.setGeometry(QtCore.QRect(270, 150, 151, 21))
        self.e3_long.setObjectName("e3_long")
        self.label_7 = QtWidgets.QLabel(MEDrone)
        self.label_7.setGeometry(QtCore.QRect(10, 150, 61, 17))
        self.label_7.setObjectName("label_7")

        self.retranslateUi(MEDrone)
        QtCore.QMetaObject.connectSlotsByName(MEDrone)

    def temizle(self, MEDrone):
        self.e1_lat.setText("")
        self.e1_long.setText("")
        self.e1_alt.setText("")

        self.e2_lat.setText("")
        self.e2_long.setText("")
        self.e2_alt.setText("")

        self.e3_lat.setText("")
        self.e3_long.setText("")
        self.e3_alt.setText("")

    def ekle(self, MEDrone):
        global hedef_list, hedef_index
        hedef_list = []
        hedef_index = []
        e1 = [self.e1_lat.text(), self.e1_long.text(), self.e1_alt.text()]
        e2 = [self.e2_lat.text(), self.e2_long.text(), self.e2_alt.text()]
        e3 = [self.e3_lat.text(), self.e3_long.text(), self.e3_alt.text()]
        ecz_list = [e1, e2, e3]
        for i in range(len(ecz_list)):
            try:
                lat = ecz_list[i][0]
                long = ecz_list[i][1]
                alt = ecz_list[i][2]

                hedef_list.append(GPSLocation(lat, long, alt).export())
                hedef_index.append(i+1)
            except ValueError:
                i += 1
        # print(hedef_list)
        if len(hedef_index) == 0:
            pass
        else:
            self.secondPage()
        # print(ecz_list)

    def retranslateUi(self, MEDrone):
        _translate = QtCore.QCoreApplication.translate
        MEDrone.setWindowTitle(_translate(
            "MEDrone", "MEDrone Otonom Uçuş Arayüzü"))
        self.pushButton.setText(_translate("MEDrone", "Konumları Ekle"))
        getButton = self.pushButton
        getButton.clicked.connect(self.ekle)
        self.label_2.setText(_translate("MEDrone", "Eczane1"))
        self.label_3.setText(_translate("MEDrone", "Enlem"))
        self.label_4.setText(_translate("MEDrone", "Boylam"))
        self.label_5.setText(_translate("MEDrone", "İrtifa"))
        self.pushButton_2.setText(_translate("MEDrone", "Temizle"))
        clear_button = self.pushButton_2
        clear_button.clicked.connect(self.temizle)
        self.label_6.setText(_translate("MEDrone", "Eczane2"))
        self.label_7.setText(_translate("MEDrone", "Eczane3"))


def start_gui(current_location, q):
    global app
    import sys
    app = QtWidgets.QApplication(sys.argv)
    home = LocationGlobalRelative(*current_location)
    MEDrone = QtWidgets.QMainWindow()
    ui = Ui_MEDrone(home, q)
    ui.setupUi(MEDrone)
    MEDrone.show()
    app.exec_()


if __name__ == "__main__":
    import sys
    q = queue.Queue()
    app = QtWidgets.QApplication(sys.argv)
    home = LocationGlobalRelative(-35.363261, 149.165230, 10)
    MEDrone = QtWidgets.QMainWindow()
    ui = Ui_MEDrone(home, q)
    ui.setupUi(MEDrone)
    MEDrone.show()
    sys.exit(app.exec_())

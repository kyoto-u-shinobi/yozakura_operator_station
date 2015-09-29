#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ref
# http://qiita.com/kiyota-yoji/items/7fe134a64177ed708fdd
# http://zbar.sourceforge.net/api/annotated.html
# http://stackoverflow.com/questions/367684/get-data-from-opengl-glreadpixelsusing-pyglet

import numpy as np
import cv2
import zbar
import PIL.Image as Image


class QRCodeSymbol(object):
    def __init__(self, symbol):
        self.count = symbol.count
        self.data = symbol.data
        self.location = symbol.location
        self.quality = symbol.quality
        self.type = symbol.type


class QRCodeDetector(object):
    def __init__(self):
        self._scanner = zbar.ImageScanner()
        self._scanner.parse_config('enable')

    def scan(self, pil_img, width, height):
        raw = pil_img.convert('L').tostring()
        image = zbar.Image(width, height, 'Y800', raw)
        self._scanner.scan(image)
        symbols = [QRCodeSymbol(symbol) for symbol in image]
        del (image)
        return symbols


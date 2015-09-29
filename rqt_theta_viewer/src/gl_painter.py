#!/usr/bin/env python

__author__ = 'matsunolab'

import OpenGL
import numpy as np

OpenGL.ERROR_CHECKING = True
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

'''
GL painting utils for theta viewer

!! Don't make it a class !!
if you do, the texture in GL doesn't work
(something happens maybe because openGL written in C++ is converted to python)
'''

# http://www.cask.cc/wp/archives/51
def map_texture_on_sphere(texture, sphere_radius, h_division, v_division):
    r = float(sphere_radius)
    h_div = int(h_division)
    v_div = int(v_division)

    glBindTexture(GL_TEXTURE_2D, texture)

    # condition settings
    _sphere = gluNewQuadric()
    gluQuadricDrawStyle(_sphere, GLU_FILL)
    gluQuadricNormals(_sphere, GLU_SMOOTH)
    gluQuadricOrientation(_sphere, GLU_OUTSIDE)  # GLU_INSIDE or GLU_OUTSIDE
    gluQuadricTexture(_sphere, GL_TRUE)

    # change texture's coordinate
    # (without this, the texture will be inside out)
    glMatrixMode(GL_TEXTURE)
    glLoadIdentity()
    glScaled(1.0, -1.0, 1.0)

    # map the texture
    glEnable(GL_TEXTURE_2D)
    gluSphere(_sphere, r, h_div,
              v_div)  # http://seesaawiki.jp/w/mikk_ni3_92/d/GLU%A4%CB%A4%E8%A4%EB%CE%A9%C2%CE%C9%BD%BC%A8
    glDisable(GL_TEXTURE_2D)

    # not necessarily??
    # glBindTexture(GL_TEXTURE_2D, 0)


def draw_basic_objects():
    glLineWidth(5)
    draw_axis(1000.0)

    glLineWidth(1)
    glColor4f(1.0, 1.0, 1.0, 1.0)
    draw_grand_gradation(200, 200, 10, 2)


def draw_axis(length):
    glColor4d(1.0, 0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(length, 0.0, 0.0)
    glEnd()

    glColor4d(0.0, 1.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(0.0, length, 0.0)
    glEnd()

    glColor4d(0.0, 0.0, 1.0, 1.0)
    glBegin(GL_LINES)
    glVertex3d(0.0, 0.0, 0.0)
    glVertex3d(0.0, 0.0, length)
    glEnd()


def draw_grand_gradation(width, depth, interval, power):
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glBegin(GL_LINES)
    for i in np.arange(-depth / 2.0, depth / 2.0, interval):
        for j in np.arange(-width / 2.0, width / 2.0, interval):
            glColor4f(1.0, 1.0, 1.0, 1.0 - pow(abs(i / (width / 2.0)), power) - pow(abs(j / (width / 2.0)), power))
            glVertex3d(j, i, 0.0)
            glVertex3d(j + interval, i, 0.0)

    for i in np.arange(-width / 2.0, width / 2.0, interval):
        for j in np.arange(-depth / 2.0, depth / 2.0, interval):
            glColor4f(1.0, 1.0, 1.0, 1.0 - pow(abs(i / (depth / 2.0)), power) - pow(abs(j / (depth / 2.0)), power))
            glVertex3d(i, j, 0.0)
            glVertex3d(i, j + interval, 0.0)
    glEnd()
    glDisable(GL_BLEND)


def draw_square_on_screen(width, height,
                          left_upper_wh_list, right_upper_wh_list,
                          left_bottom_wh_list, right_bottom_wh_list,
                          rgb=(1.0, 0.0, 0.0), alpha=0.8):
    glPushMatrix()
    glLoadIdentity()
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, width, 0, height)
    glEnable(GL_BLEND)
    glColor4d(rgb[0], rgb[1], rgb[2], alpha)
    glBegin(GL_QUADS)
    for lu, ru, rb, lb in zip(left_upper_wh_list, right_upper_wh_list,
                              right_bottom_wh_list, left_bottom_wh_list):
        glVertex3d(lu[0], lu[1], 0)
        glVertex3d(ru[0], ru[1], 0)
        glVertex3d(rb[0], rb[1], 0)
        glVertex3d(lb[0], lb[1], 0)
    glEnd()
    glDisable(GL_BLEND)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)
    glPopMatrix()


def draw_texts(width, height, texts, wh_list):
    """
    doesn't work
    you have to add glutInit
    """
    glPushMatrix()

    glLoadIdentity()
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    gluOrtho2D(0, width, 0, height)

    for text, wh in zip(texts, wh_list):
        glTranslated(wh[0], wh[1], 0.0)
        glScaled(0.2, 0.2, 0.2)
        for char in str(text):
            glutStrokeCharacter(GLUT_STROKE_ROMAN, char)

    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)

    glPopMatrix()


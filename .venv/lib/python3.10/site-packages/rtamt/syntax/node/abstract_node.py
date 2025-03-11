# -*- coding: utf-8 -*-
"""
Created on Mon Sep 23 2019

@author: Dejan Nickovic
"""
from abc import ABCMeta

class AbstractNode:
    """
    Abstract Node: tree-like data structure containing
    arbitrary specifications
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        self.in_vars = []
        self.out_vars = []
        self.children = list()
        self.interpreter = None
        self.name = ''
        self.node = None


    def add_child(self, child):
        self.children.append(child)

    def accept(self, visitor):
        """accept: recursive function needed to implement node visitors
        Inputs:
        visitor - Visitor object
        """
        for child in self.children:
            child.accept(visitor)

    @property
    def interpreter(self):
        """Getter for the online_interpreter"""
        return self.__interpreter

    @interpreter.setter
    def interpreter(self, interpreter):
        """Setter for the online_interpreter"""
        self.__interpreter = interpreter

    @property
    def name(self):
        """Getter for the name"""
        return self.__name

    @name.setter
    def name(self, name):
        """Setter for the name"""
        self.__name = name

    @property
    def node(self):
        """Getter for the node"""
        return self.__node

    @node.setter
    def node(self, node):
        """Setter for the horizon"""
        self.__node = node

    @property
    def children(self):
        return self.__children

    @children.setter
    def children(self, children):
        self.__children = children

    @property
    def in_vars(self):
        """Getter for the in_vars"""
        return self.__in_vars

    @in_vars.setter
    def in_vars(self, in_vars):
        """Setter for the in_vars"""
        self.__in_vars = in_vars

    @property
    def out_vars(self):
        """Getter for the out_vars"""
        return self.__out_vars

    @out_vars.setter
    def out_vars(self, out_vars):
        """Setter for the out_vars"""
        self.__out_vars = out_vars

    def __repr__(self):
        '''Returns representation of the object'''
        return self.__class__.__name__

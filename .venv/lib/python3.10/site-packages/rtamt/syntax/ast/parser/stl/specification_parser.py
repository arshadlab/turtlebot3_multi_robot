# -*- coding: utf-8 -*-
from rtamt.syntax.ast.parser.stl.parser_visitor import StlAstParserVisitor
from rtamt.antlr.parser.stl.StlLexer import StlLexer
from rtamt.antlr.parser.stl.StlParser import StlParser
from rtamt.syntax.ast.parser.abstract_ast_parser import ast_factory
from rtamt.antlr.parser.stl.error.parser_error_listener import STLParserErrorListener


def StlAst():
    antrlLexerType = globals()['StlLexer']
    antrlParserType = globals()['StlParser']
    parserErrorListenerType = globals()['STLParserErrorListener']   #optional
    stlAst = ast_factory(StlAstParserVisitor)(antrlLexerType, antrlParserType, parserErrorListenerType)
    return stlAst

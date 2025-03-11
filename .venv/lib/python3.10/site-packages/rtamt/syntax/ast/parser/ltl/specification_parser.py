# -*- coding: utf-8 -*-

from rtamt.antlr.parser.ltl.LtlLexer import LtlLexer
from rtamt.antlr.parser.ltl.LtlParser import LtlParser
from rtamt.antlr.parser.ltl.error.parser_error_listener import LTLParserErrorListener
from rtamt.syntax.ast.parser.ltl.parser_visitor import LtlAstParserVisitor
from rtamt.syntax.ast.parser.abstract_ast_parser import ast_factory


def LtlAst():
    #TODO we may explain why we need to write like this.
    antrlLexerType = globals()['LtlLexer']
    antrlParserType = globals()['LtlParser']
    parserErrorListenerType = globals()['LTLParserErrorListener']   #optional
    ltLAst = ast_factory(LtlAstParserVisitor)(antrlLexerType, antrlParserType, parserErrorListenerType)
    return ltLAst

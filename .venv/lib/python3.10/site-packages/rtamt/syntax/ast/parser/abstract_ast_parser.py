from abc import ABCMeta
import logging
import importlib
from antlr4 import *
from antlr4.InputStream import InputStream
from antlr4.error.ErrorListener import ErrorListener

from rtamt.exception.exception import RTAMTException

class AbstractAst:
    """An abstract class for AST parser

    Attributes:
        name : String

        modular_spec : String - specification text
        spec : String - specification text

        vars : set(String) - set of variable names
        free_vars : set(String) - set of free variable names

        var_subspec_dict : dict(String, AbstractNode) - dictionary that maps variable names to the AST
        var_object_dict : dict(String, double) - dictionary that maps variable names to their value
        modules : dict(String,String) - dictionary that maps module paths to module names
        var_type_dict : dict(String, String) - dictionary that maps var names to var types
        var_io_dict : dict(String, String) - dictionary that maps var names to var io signature
        const_type_dict : dict(String, String) - dictionary mapping const var names to var types
        const_val_dict : dict(String, String) - dictionary mapping const var names to var vals encoded as strings

        ast : Node - pointer to the specification parse tree

    Methods
        parse - parse the specification

        declare_var - declare variable in spec
        declare_const - declare const variable in spec
        add_sub_spec - add sub spec

    """


    __metaclass__ = ABCMeta

    def __init__(self, antrlLexerType, antrlParserType, parserErrorListenerType = None):

        # Class of lexser, parser, paserVisitor
        #TODO we need class check which inherits expected abstract class.
        self.antrlLexerType = antrlLexerType
        self.antrlParserType = antrlParserType
        self.parserErrorListenerType = parserErrorListenerType

        # Attributes
        self.name = 'Abstract Specification'
        self.spec = None
        self.modular_spec = ''

        self.vars = set()
        self.in_vars = set()
        self.out_vars = set()
        self.free_vars = set()

        self.var_subspec_dict = dict()
        self.var_object_dict = dict()
        self.var_type_dict = dict()
        self.var_io_dict = dict()
        self.const_type_dict = dict()
        self.const_val_dict = dict()

        self.modules = dict()


        #TODO perhaps, we may omit it from AST.
        self.var_topic_dict = dict()

        self.S_UNIT = int(1000000000)
        self.MS_UNIT = int(1000000)
        self.US_UNIT = int(1000)
        self.NS_UNIT = int(1)

        self.U = {
            's': self.S_UNIT,
            'ms': self.MS_UNIT,
            'us': self.US_UNIT,
            'ns': self.NS_UNIT
        }

        # Default unit
        self.unit = 's'

        self.sampling_period = int(1)
        self.sampling_period_unit = 's'

        self.specs = []

        self.out_var = None
        self.out_var_field = None

        #TODO Tom did not understand it well.
        # 1) Is it only stl? I don't think so
        # 2) Maybe we may consider where we may handle C++/python switch. Maybe semantics layer.
        #io_type_name = 'rtamt.lib.rtamt_stl_library_wrapper.stl_io_type'
        #comp_op_name = 'rtamt.lib.rtamt_stl_library_wrapper.stl_comp_op'
        #if self.spec.language == Language.PYTHON:
        #    io_type_name = 'rtamt.semantics.enumerations.io_type'
        #    comp_op_name = 'rtamt.semantics.enumerations.comp_op'
        io_type_name = 'rtamt.semantics.enumerations.io_type'
        comp_op_name = 'rtamt.semantics.enumerations.comp_op'

        self.io_type_mod = __import__(io_type_name, fromlist=[''])
        self.comp_op_mod = __import__(comp_op_name, fromlist=[''])

        return

    def parse(self):
        if self.spec is None:
            raise RTAMTException('STL specification if empty')

        #TODO How to handle sub-formulas?
        entire_spec = self.modular_spec + self.spec
        input_stream = InputStream(entire_spec)
        lexer = self.antrlLexerType(input_stream)
        if not isinstance(lexer, Lexer):
            raise RTAMTException('{} is not ANTRL4 Lexer'.format(lexer.__class__.__name__))
        stream = CommonTokenStream(lexer)
        parser = self.antrlParserType(stream)
        if not isinstance(parser, Parser):
            raise RTAMTException('{} is not ANTRL4 Parser'.format(parser.__class__.__name__))
        if self.parserErrorListenerType != None:
            parser._listeners = [self.parserErrorListenerType()]
            if not isinstance(parser._listeners[0], ErrorListener):
                raise RTAMTException('{} is not ANTRL4 ErrorListener'.format(parser._listeners[0].__class__.__name__))
        ctx = parser.specification_file()
        self.visit(ctx.specification())
        return

    @property
    def out_var(self):
        return self.__out_var

    @out_var.setter
    def out_var(self, out_var):
        self.__out_var = out_var

    @property
    def out_var_field(self):
        return self.__out_var_field

    @out_var_field.setter
    def out_var_field(self, out_var_field):
        self.__out_var_field = out_var_field

    @property
    def spec(self):
        return self.__spec

    @spec.setter
    def spec(self, spec):
        self.__spec = spec

    @property
    def specs(self):
        return self.__specs

    @specs.setter
    def specs(self, specs):
        self.__specs = specs

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def free_vars(self):
        return self.__free_vars

    @free_vars.setter
    def free_vars(self, free_vars):
        self.__free_vars = free_vars

    @property
    def vars(self):
        return self.__vars

    @vars.setter
    def vars(self, vars):
        self.__vars = vars

    @property
    def modules(self):
        return self.__modules

    @modules.setter
    def modules(self, modules):
        self.__modules = modules

    def add_var(self, var):
        self.vars.add(var)

    def get_value(self, var_name):
        return self.var_object_dict[var_name]

    def add_sub_spec(self, sub_spec):
        self.modular_spec = self.modular_spec + sub_spec + '\n'

    def create_var_from_name(self, var_name):
        var = None
        var_type = self.var_type_dict[var_name]
        if var_type.encode('utf-8') == 'float'.encode('utf-8'):
            var = float()
        elif var_type.encode('utf-8') == 'int'.encode('utf-8'):
            var = int()
        elif var_type.encode('utf-8') == 'complex'.encode('utf-8'):
            var = complex()
        else:
            try:
                var_module = self.modules[var_type]
                class_ = getattr(var_module, var_type)
                var = class_()
            except KeyError:
                raise RTAMTException('The type {} does not seem to be imported.'.format(var_type))
        return var

    def declare_var(self, var_name, var_type):
        if var_name in self.vars:
            logging.warning(
                'Variable {} was already declared. It is now overriden with the new declaration.'.format(var_name))

        # Associate to variable name 'var' its type 'type'
        self.var_type_dict[var_name] = var_type

        # Add variable name 'var' to the set of variables
        self.add_var(var_name)
        self.free_vars.add(var_name)
        instance = self.create_var_from_name(var_name)
        self.var_object_dict[var_name] = instance

        # Add the default variable topic to var
        #TODO perhaps, we may omit it from AST.
        self.var_topic_dict[var_name] = 'rtamt/{}'.format(var_name)

        self.var_io_dict[var_name] = 'output'

    def declare_const(self, const_name, const_type, const_val):
        if const_name in self.vars:
            raise RTAMTException('Constant {} already declared'.format(const_name))

        self.const_type_dict[const_name] = const_type
        self.const_val_dict[const_name] = const_val
        self.vars.add(const_name)

    def import_module(self, from_name, module_name):
        try:
            module = importlib.import_module(from_name)
            self.modules[module_name] = module
        except ImportError:
            raise RTAMTException('The module {} cannot be loaded'.format(from_name))

    def set_var_topic(self, var_name, var_topic):
        if not var_name in self.vars:
            logging.warning(
                'The variable {0} is not declared. Setting its topic name to {1} is ignored.'.format(var_name,
                                                                                                     var_topic))
        else:
            topic = self.var_topic_dict[var_name]
            self.var_topic_dict[var_name] = var_topic

    def set_var_io_type(self, var_name, var_iotype):
        if not var_name in self.vars:
            logging.warning('The variable {} does not exist'.format(var_name))
        else:
            if var_iotype == 'input':
                self.add_input_var(var_name)
                self.remove_output_var(var_name)
                self.var_io_dict[var_name] = 'input'
            elif var_iotype == 'output':
                self.add_output_var(var_name)
                self.remove_input_var(var_name)
                self.var_io_dict[var_name] = 'output'
            else:
                self.remove_input_var(var_name)
                self.remove_output_var(var_name)
                self.var_io_dict[var_name] = 'undefined'

    def add_input_var(self, input_var):
        self.in_vars.add(input_var)

    def remove_input_var(self, var):
        self.in_vars.discard(var)

    def add_output_var(self, output_var):
        self.out_vars.add(output_var)

    def remove_output_var(self, var):
        self.out_vars.discard(var)

def ast_factory(AstParserVisitor):
    class Ast(AbstractAst, AstParserVisitor):
        def __init__(self, antrlLexerType, antrlParserType, parserErrorListenerType=None):
            AbstractAst.__init__(self, antrlLexerType, antrlParserType, parserErrorListenerType)
            AstParserVisitor.__init__(self)
    return Ast
import logging
import importlib

from rtamt.spec.abstract_specification import AbstractSpecification
from rtamt.syntax.ast.parser.ltl.specification_parser import LtlAstParserVisitor

from rtamt.exception.exception import RTAMTException

from rtamt.pastifier.ltl.pastifier import LtlPastifier
from rtamt.interpreter.ltl.online_interpreter import LTLInterpreter
from rtamt.reset.ltl.reset import LTLReset
from rtamt.semantics.enumerations.options import *


class LTLDiscreteTimeSpecification(AbstractSpecification):
    """A class used as a container for STL specifications

    Attributes:
        name : String

        vars : set(String) - set of variable names
        free_vars : set(String) - set of free variable names

        sampling_period : int - size of the sampling period


        var_object_dict : dict(String,AbstractNode) - dictionary that maps variable names to their Node instances
        modules : dict(String,String) - dictionary that maps module paths to module names

        top : AbstractNode - pointer to the specification parse tree

        online_interpreter : AbstractInterpreter - pointer to the object that implements the monitoring algorithm

        update_counter : int
        previous_time : float
        sampling_violation_counter : int

    """

    def __init__(self, semantics=Semantics.STANDARD, language=Language.PYTHON):
        """Constructor for STL Specification"""
        super(LTLDiscreteTimeSpecification, self).__init__()
        self.name = 'LTL Specification'

        self.reseter = LTLReset()
        self.semantics = semantics
        self.language = language

        self.in_vars = set()
        self.out_vars = set()

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

    def import_module(self, from_name, module_name):
        try:
            module = importlib.import_module(from_name)
            self.modules[module_name] = module
        except ImportError:
            raise RTAMTException('The module {} cannot be loaded'.format(from_name))

    def declare_const(self, const_name, const_type, const_val):
        if const_name in self.vars:
            raise RTAMTException('Constant {} already declared'.format(const_name))

        self.const_type_dict[const_name] = const_type
        self.const_val_dict[const_name] = const_val
        self.vars.add(const_name)

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
        self.var_topic_dict[var_name] = 'rtamt/{}'.format(var_name)

        self.var_io_dict[var_name] = 'output'

    def set_var_topic(self, var_name, var_topic):
        if not var_name in self.vars:
            logging.warning(
                'The variable {0} is not declared. Setting its topic name to {1} is ignored.'.format(var_name,
                                                                                                     var_topic))
        else:
            topic = self.var_topic_dict[var_name]
            self.var_topic_dict[var_name] = var_topic

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

    @property
    def semantics(self):
        return self.__semantics

    @semantics.setter
    def semantics(self, semantics):
        self.__semantics = semantics

    @property
    def in_vars(self):
        return self.__in_vars

    @in_vars.setter
    def in_vars(self, in_vars):
        self.__in_vars = in_vars

    @property
    def out_vars(self):
        return self.__out_vars

    @out_vars.setter
    def out_vars(self, out_vars):
        self.__out_vars = out_vars

    # Parses the STL property
    # string can be either file path containing the STL property
    # or the textual property itself
    def parse(self):
        # Create the visitor for the actual spec nodes
        parser = LtlAstParserVisitor(self)
        self.top = parser.parse()

    def pastify(self):
        # Translate bounded future STL to past STL
        pastifier = LTLPastifier()
        self.top.accept(pastifier)

        # evaluate modular sub-specs
        for key in self.var_subspec_dict:
            node = self.var_subspec_dict[key]
            node.accept(pastifier)
            node = pastifier.pastify(node)
            self.var_subspec_dict[key] = node

        past = pastifier.pastify(self.top)
        self.top = past

    def update(self, timestamp, list_inputs):
        # timestamp - float
        # inputs - list of [var name, var value] pairs
        # Example:
        # update(3.48, [['a', 2.2], ['b', 3.3]])

        if self.online_interpreter == None:
            self.online_interpreter = LTLInterpreter(self)
            self.top.accept(self.online_interpreter)

        # update the value of every input variable
        for inp in list_inputs:
            var_name = inp[0]
            var_value = inp[1]
            self.var_object_dict[var_name] = var_value

        # evaluate modular sub-specs
        for key in self.var_subspec_dict:
            node = self.var_subspec_dict[key]
            out = self.online_interpreter.evaluate(node, [])
            self.var_object_dict[key] = out

        # The evaluation done wrt the discrete counter (logical time)
        out = self.online_interpreter.evaluate(self.top, [])

        return out

    def evaluate(self, *args, **kargs):
        pass

    def reset(self):
        if self.online_interpreter is not None:
            self.reseter.node_monitor_dict = self.online_interpreter.node_monitor_dict
            self.top.accept(self.reseter)
            self.reseter.reset(self.top)
            self.update_counter = int(0);
            self.previous_time = float(0.0);
            self.sampling_violation_counter = int(0);

    # def offline(self, dataset):
    #     counter = 0
    #     prev_signal_length = 0
    #     signal_length = 0
    #     out = 0
    #
    #     for var_name in dataset:
    #         signal_length = len(dataset[var_name])
    #         if counter > 0 and not (signal_length == prev_signal_length):
    #             raise STLOfflineException('Input signals have different length')
    #         prev_signal_length = signal_length
    #         counter = counter + 1
    #
    #     for i in range(signal_length):
    #         signal_snapshot = []
    #         counter = 0
    #         prev_time = 0
    #         for var_name in dataset:
    #             signal = dataset[var_name]
    #             sample = signal[i]
    #             time = sample[0]
    #             value = sample[1]
    #             if counter > 0 and not (time == prev_time):
    #                 raise STLOfflineException('The time indices do not agree')
    #             signal_snapshot.append((var_name, value))
    #             counter = counter + 1
    #             prev_time = time
    #         out = self.update(time, signal_snapshot)
    #
    #     return out

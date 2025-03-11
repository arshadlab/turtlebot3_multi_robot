import os
from abc import ABCMeta

from rtamt.semantics.abstract_discrete_time_online_interpreter import AbstractDiscreteTimeOnlineInterpreter
from rtamt.semantics.abstract_dense_time_online_interpreter import AbstractDenseTimeOnlineInterpreter
from rtamt.semantics.abstract_discrete_time_offline_interpreter import AbstractDiscreteTimeOfflineInterpreter
from rtamt.semantics.abstract_dense_time_offline_interpreter import AbstractDenseTimeOfflineInterpreter

from rtamt.semantics.discrete_time_interpreter import DiscreteTimeInterpreter

from rtamt.exception.exception import RTAMTException


class AbstractSpecification(object):
    __metaclass__ = ABCMeta

    def __init__(self, ast):
        self.name = 'Abstract Specification'
        self.ast = ast
        self.interpreter = None
        self.set_ast_flag = False # It is for interpreter is set ast or not.
        self.out_var = ''
        self.out_var_field = ''
        self.var_topic_dict = dict()
        self.free_vars = set()
        self.var_object_dict = dict()

        #TODO we need to move it to RTAMT4ROS as wrapper
        self.modules = dict()

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def spec(self):
        return self.ast.spec

    @spec.setter
    def spec(self, spec):
        self.ast.spec = spec

    # forwarding to ast
    def add_var(self, var):
        self.ast.vars.add(var)

    def get_value(self, var_name):
        return self.ast.get_value(var_name)

    def add_sub_spec(self, sub_spec):
        self.ast.add_sub_spec(sub_spec)

    def set_var_io_type(self, var, io_type):
        self.ast.set_var_io_type(var, io_type)

    def declare_var(self, var_name, var_type):
        self.ast.declare_var(var_name, var_type)

    def declare_const(self, const_name, const_type, const_val):
        self.ast.declare_const(const_name, const_type, const_val)

    @property
    def out_var(self):
        return self.ast.out_var

    @out_var.setter
    def out_var(self, out_var):
        pass

    @property
    def out_var_field(self):
        return self.ast.out_var_field

    @out_var_field.setter
    def out_var_field(self, out_var_field):
        pass

    @property
    def var_topic_dict(self):
        return self.ast.var_topic_dict

    @var_topic_dict.setter
    def var_topic_dict(self, var_topic_dict):
        pass

    @property
    def var_object_dict(self):
        return self.ast.var_object_dict

    @var_object_dict.setter
    def var_object_dict(self, var_object_dict):
        self.__var_object_dict = var_object_dict
        self.ast.var_object_dict = self.var_object_dict

    @property
    def free_vars(self):
        return self.ast.free_vars

    @free_vars.setter
    def free_vars(self, free_vars):
        pass

    def modules(self, modules): #TODO: send syntax layer (ast)?
        return self.ast.modules(modules)

    def import_module(self, from_name, module_name): #TODO: send syntax layer (ast)?
        self.ast.import_module(from_name, module_name)

    def set_var_topic(self, var_name, var_topic):
        self.ast.set_var_topic(var_name, var_topic)

    def parse(self):
        self.ast.parse()

    # forwarding to interpreter
    def set_sampling_period(self, sampling_period=int(1), unit='s', tolerance=float(0.1)):
        if hasattr(self, 'online_interpreter'):
            if isinstance(self.online_interpreter, DiscreteTimeInterpreter):
                self.online_interpreter.set_sampling_period(sampling_period, unit, tolerance)
            else:
                RTAMTException('time_unit_transformer() allowed only discrete time')

        if hasattr(self, 'offline_interpreter'):
            if isinstance(self.offline_interpreter, DiscreteTimeInterpreter):
                self.offline_interpreter.set_sampling_period(sampling_period, unit, tolerance)
            else:
                RTAMTException('time_unit_transformer() allowed only discrete time')

    def get_sampling_frequency(self):
        if hasattr(self, 'online_interpreter'):
            if isinstance(self.online_interpreter, DiscreteTimeInterpreter):
                return self.online_interpreter.get_sampling_frequency()
            else:
                RTAMTException('time_unit_transformer() allowed only discrete time')
        if hasattr(self, 'offline_interpreter'):
            if isinstance(self.offline_interpreter, DiscreteTimeInterpreter):
                return self.offline_interpreter.get_sampling_frequency()
            else:
                RTAMTException('time_unit_transformer() allowed only discrete time')

    @property
    def sampling_violation_counter(self):
        if hasattr(self, 'online_interpreter'):
            if isinstance(self.online_interpreter, DiscreteTimeInterpreter):
                return self.online_interpreter.sampling_violation_counter
            else:
                RTAMTException('only discrete time has sampling_violation_counter')
        if hasattr(self, 'offline_interpreter'):
            if isinstance(self.offline_interpreter, DiscreteTimeInterpreter):
                return self.offline_interpreter.sampling_violation_counter
            else:
                RTAMTException('only discrete time has sampling_violation_counter')

    @property
    def sampling_tolerance(self):
        if hasattr(self, 'online_interpreter'):
            if isinstance(self.online_interpreter, DiscreteTimeInterpreter):
                return self.online_interpreter.sampling_tolerance
            else:
                RTAMTException('only discrete time has sampling_tolerance')
        if hasattr(self, 'offline_interpreter'):
            if isinstance(self.offline_interpreter, DiscreteTimeInterpreter):
                return self.offline_interpreter.sampling_tolerance
            else:
                RTAMTException('only discrete time has sampling_tolerance')

    #TODO we need to move it to RTAMT4ROS as wrapper
    @property
    def publish_var(self):
        return self.__publish_var

    @publish_var.setter
    def publish_var(self, publish_var):
        self.__publish_var = publish_var

    @property
    def publish_var_field(self):
        return self.__publish_var_field

    @publish_var_field.setter
    def publish_var_field(self, publish_var_field):
        self.__publish_var_field = publish_var_field

    #TODO we are wondering. put add it to issue comment
    def add_input_var(self, input_var):
        self.in_vars.add(input_var)

    def remove_input_var(self, var):
        self.in_vars.discard(var)

    def add_output_var(self, output_var):
        self.out_vars.add(output_var)

    def remove_output_var(self, var):
        self.out_vars.discard(var)

    def add_op(self, op):
        self.ops.add(op)


    #TODO goto Syntax while keeping in Spec too.
    def get_spec_from_file(self, path):
        """Opens a text file and returns its content as a string
        Parameters:
            path : String - path to the filename
        Returns
            out : String - file content
        """
        out = None
        if os.path.exists(path):
            f = open(path, "r")
            out = f.read()
            f.close()
        else:
            raise RTAMTException('The file {} does not exist.'.format(path))
        return out


class DiscreteTimeOfflineInterpreter(object):
    pass


class AbstractOfflineSpecification(AbstractSpecification):
    def __init__(self, ast, offlineInterpreter):
        AbstractSpecification.__init__(self, ast)
        self.name = 'Abstract Offline Specification'
        self.offline_interpreter = offlineInterpreter

    # forwarding to interpreter
    def evaluate(self, *args, **kwargs):
        if self.set_ast_flag != True:
            self.offline_interpreter.set_ast(self.ast)
            self.set_ast_flag = True

        #TODO we may make it consistent with interpreter class.
        if isinstance(self.offline_interpreter, AbstractDenseTimeOfflineInterpreter):
            if len(args) == 0:
                raise Exception()
            elif len(args) == 1:
                dataset = [args[0]]
            else:
                dataset = []
                for i in args:
                    dataset.append(i)
            return self.offline_interpreter.evaluate(dataset)
        elif isinstance(self.offline_interpreter, AbstractDiscreteTimeOfflineInterpreter):
            dataset = args[0]
            return self.offline_interpreter.evaluate(dataset)
        else:
            raise Exception('Wrong interpreter!')


class AbstractOnlineSpecification(AbstractSpecification):
    def __init__(self, ast, onlineInterpreter, pastifier=None):
        AbstractSpecification.__init__(self, ast)
        self.name = 'Abstract Online Specification'
        self.online_interpreter = onlineInterpreter
        self.pastifier = pastifier

    # forwarding pastify
    def pastify(self):
        self.ast = self.pastifier.pastify(self.ast)

    # forwarding to interpreter
    def update(self, *args, **kwargs):
        if self.set_ast_flag != True:
            self.online_interpreter.set_ast(self.ast)
            self.set_ast_flag = True

        #TODO we may make it consistent with interpreter class.
        if isinstance(self.online_interpreter, AbstractDenseTimeOnlineInterpreter):
            if len(args) == 0:
                raise Exception()
            elif len(args) == 1:
                dataset = [args[0]]
            else:
                dataset = []
                for i in args:
                    dataset.append(i)
            return self.online_interpreter.update(dataset)
        elif isinstance(self.online_interpreter, AbstractDiscreteTimeOnlineInterpreter):
            i = args[0]
            dataset = args[1]
            return self.online_interpreter.update(i, dataset)
        else:
            pass

    def final_update(self, *args, **kwargs):
        if self.set_ast_flag != True:
            self.online_interpreter.set_ast(self.ast)
            self.set_ast_flag = True

        #TODO we may make it consistent with interpreter class.
        if len(args) == 0:
            raise Exception()
        elif len(args) == 1:
            dataset = [args[0]]
        else:
            dataset = []
            for i in args:
                dataset.append(i)

        return self.online_interpreter.final_update(dataset)

    def reset(self):
        self.online_interpreter.reset()


# we would not recomend to use it
# Please note that. Even the class have both evaluate and update, calling both with same instance is not expected.
class AbstractOfflineOnlineSpecification(AbstractOfflineSpecification, AbstractOnlineSpecification):
    def __init__(self, ast, offlineInterpreter, onlineInterpreter, pastifier=None):
        AbstractOfflineSpecification.__init__(self, ast, offlineInterpreter)
        AbstractOnlineSpecification.__init__(self, ast, onlineInterpreter, pastifier)
        self.name = 'Abstract Offline Online Specification'

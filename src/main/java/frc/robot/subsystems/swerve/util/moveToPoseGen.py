
imports = [
    "java.util.function.Consumer",
    "java.util.function.DoubleSupplier",
    "java.util.function.Supplier",
    "edu.wpi.first.math.geometry.Pose2d",
    "edu.wpi.first.math.kinematics.ChassisSpeeds",
    "edu.wpi.first.math.util.Units",
    "edu.wpi.first.wpilibj.event.EventLoop",
    "frc.robot.Constants",
    "frc.robot.subsystems.swerve.Swerve"
]

default_fields = [
    ["Swerve", "swerve"],
    ["Consumer<ChassisSpeeds>", "robotRelativeConsumer"],
]

required_fields = [
    ["Supplier<Pose2d>", "target", ["Pose2d", "targetConst", "() -> targetConst"]],
]

optional_fields = [
    ["EventLoop", "eventLoop", "null"],
    ["DoubleSupplier", "maxSpeed", "() -> Constants.Swerve.autoMaxSpeed", ["double", "maxSpeedConst", "() -> maxSpeedConst"]],
    ["boolean", "flipForRed", "true"],
    ["double", "translationTolerance", "0.5"],
    ["double", "rotationTolerance", "Units.degreesToRadians(5)"]
]

class Fields:
    def __init__(self, name, default_fields, required_fields, optional_fields, permutation, constructor):
        self.name = name
        self.permutation = permutation
        self.required_set = [((permutation >> j) & 0x1) != 0 for j in range(0, len(required_fields))]
        self.optional_set = [((permutation >> (j + len(required_fields))) & 0x1) != 0 for j in range(0, len(optional_fields))]
        self.default_fields = default_fields
        self.required_fields = required_fields
        self.optional_fields = optional_fields
        self.constructor = constructor

    def next(self, index):
        return Fields(self.name, self.default_fields, self.required_fields, self.optional_fields, self.permutation | (1 << index), self.constructor)

    def is_full(self):
        return all(self.required_set) and all(self.optional_set)

    def could_finish(self):
        return all(self.required_set)

    def write_constructor(self, f, indent, public):
        if public:
            f.write('\n{}/** generated */'.format(indent))
        f.write('\n{}{} {}('.format(indent, "public" if public else "private", self.builder_state_name() if not public else "{}Builder".format(self.name)))
        has_args = False
        for i in range(0, len(self.default_fields)):
            if has_args:
                f.write(',')
            f.write('{} {}'.format(self.default_fields[i][0], self.default_fields[i][1]))
            has_args = True
        for i in range(0, len(self.required_fields)):
            if self.required_set[i]:
                if has_args:
                    f.write(',')
                f.write('{} {}'.format(self.required_fields[i][0], self.required_fields[i][1]))
                has_args = True
        for i in range(0, len(self.optional_fields)):
            if self.optional_set[i]:
                if has_args:
                    f.write(',')
                f.write('{} {}'.format(self.optional_fields[i][0], self.optional_fields[i][1]))
                has_args = True
        f.write('){\n')
        for i in range(0, len(self.default_fields)):
            f.write('{}    this.{} = {};\n'.format(indent, self.default_fields[i][1], self.default_fields[i][1]))
        for i in range(0, len(self.required_fields)):
            if self.required_set[i]:
                f.write('{}    this.{} = {};\n'.format(indent, self.required_fields[i][1], self.required_fields[i][1]))
        for i in range(0, len(self.optional_fields)):
            if self.optional_set[i]:
                f.write('{}    this.{} = {};\n'.format(indent, self.optional_fields[i][1], self.optional_fields[i][1]))
        f.write('{}}}\n'.format(indent))
        pass

    def write_fields(self, f, indent):
        for i in range(0, len(self.default_fields)):
            f.write('{}private final {} {};\n'.format(indent, self.default_fields[i][0], self.default_fields[i][1]))
        for i in range(0, len(self.required_fields)):
            if self.required_set[i]:
                f.write('{}private final {} {};\n'.format(indent, self.required_fields[i][0], self.required_fields[i][1]))
        for i in range(0, len(self.optional_fields)):
            if self.optional_set[i]:
                f.write('{}private final {} {};\n'.format(indent, self.optional_fields[i][0], self.optional_fields[i][1]))
        pass

    def write_methods(self, f, indent):
        if self.could_finish():
            f.write('\n{}/** generated */\n'.format(indent))
            f.write('{}public {} finish() {{\n'.format(indent, self.name))
            args = []
            for i in range(0, len(self.default_fields)):
                args.append(self.default_fields[i][1])
            for i in range(0, len(self.required_fields)):
                args.append(self.required_fields[i][1])
            for i in range(0, len(self.optional_fields)):
                if self.optional_set[i]:
                    args.append(self.optional_fields[i][1])
                else:
                    args.append(self.optional_fields[i][2])
            f.write('{}    return {};\n'.format(indent, self.constructor.format(', '.join(args))))
            f.write('{}}}\n'.format(indent))

        for [offset, fields] in [[0, self.required_fields], [len(self.required_fields), self.optional_fields]]:
            for i in range(0, len(fields)):
                if (self.permutation >> (i + offset)) & 0x1 != 0:
                    continue
                ret = self.next(i + offset)
                if ret.is_full():
                    f.write('\n{}/** generated */\n'.format(indent))
                    f.write('{}public {} {}({} {}) {{\n'.format(indent, self.name, fields[i][1], fields[i][0], fields[i][1]))
                    args = []
                    for j in range(0, len(self.default_fields)):
                        args.append(self.default_fields[j][1])
                    for j in range(0, len(self.required_fields)):
                        args.append(self.required_fields[j][1])
                    for j in range(0, len(self.optional_fields)):
                        args.append(self.optional_fields[j][1])
                    f.write('{}    return {};\n'.format(indent, self.constructor.format(', '.join(args))))
                    f.write('{}}}\n'.format(indent))

                    if type(fields[i][-1]) == list:
                        f.write('\n{}/** generated */\n'.format(indent))
                        f.write('{}public {} {}({} {}) {{\n'.format(indent, self.name, fields[i][1], fields[i][-1][0], fields[i][-1][1]))
                        f.write('{}    {} {} = {};\n'.format(indent, fields[i][0], fields[i][1], fields[i][-1][2]))
                        args = []
                        for j in range(0, len(self.default_fields)):
                            args.append(self.default_fields[j][1])
                        for j in range(0, len(self.required_fields)):
                            args.append(self.required_fields[j][1])
                        for j in range(0, len(self.optional_fields)):
                            args.append(self.optional_fields[j][1])
                        f.write('{}    return {};\n'.format(indent, self.constructor.format(', '.join(args))))
                        f.write('{}}}\n'.format(indent))
                        pass
                else:
                    f.write('\n{}/** generated */\n'.format(indent))
                    f.write('{}public {} {}({} {}) {{\n'.format(indent, ret.builder_state_name(), fields[i][1], fields[i][0], fields[i][1]))
                    f.write('{}    return new {}('.format(indent, ret.builder_state_name()))
                    written_arg = False
                    for j in range(0, len(self.default_fields)):
                        if written_arg:
                            f.write(', ')
                        f.write(self.default_fields[j][1])
                        written_arg = True
                    for j in range(0, len(self.required_fields)):
                        if not ret.required_set[j]:
                            continue
                        if written_arg:
                            f.write(', ')
                        f.write(self.required_fields[j][1])
                        written_arg = True
                    for j in range(0, len(self.optional_fields)):
                        if not ret.optional_set[j]:
                            continue
                        if written_arg:
                            f.write(', ')
                        f.write(self.optional_fields[j][1])
                        written_arg = True
                    f.write(');\n')
                    f.write('{}}}\n'.format(indent))

                    if type(fields[i][-1]) == list:
                        f.write('\n{}/** generated */\n'.format(indent))
                        f.write('{}public {} {}({} {}) {{\n'.format(indent, ret.builder_state_name(), fields[i][1], fields[i][-1][0], fields[i][-1][1]))
                        f.write('{}    {} {} = {};\n'.format(indent, fields[i][0], fields[i][1], fields[i][-1][2]))
                        f.write('{}    return new {}('.format(indent, ret.builder_state_name()))
                        written_arg = False
                        for j in range(0, len(self.default_fields)):
                            if written_arg:
                                f.write(', ')
                            f.write(self.default_fields[j][1])
                            written_arg = True
                        for j in range(0, len(self.required_fields)):
                            if not ret.required_set[j]:
                                continue
                            if written_arg:
                                f.write(', ')
                            f.write(self.required_fields[j][1])
                            written_arg = True
                        for j in range(0, len(self.optional_fields)):
                            if not ret.optional_set[j]:
                                continue
                            if written_arg:
                                f.write(', ')
                            f.write(self.optional_fields[j][1])
                            written_arg = True
                        f.write(');\n')
                        f.write('{}}}\n'.format(indent))

    def builder_state_name(self):
        name = "BuilderState"
        for item in self.required_set:
            name = "{}{}".format(name, "1" if item else "0")
        for item in self.optional_set:
            name = "{}{}".format(name, "1" if item else "0")
        return name

num_fields = len(required_fields) + len(optional_fields)

with open('MoveToPoseBuilder.java', 'w') as f:
    f.write('package frc.robot.subsystems.swerve.util;\n\n')

    for imp in imports:
        f.write('import {};\n'.format(imp))

    f.write('\n/** generated */\n')
    f.write('public class MoveToPoseBuilder {\n')
    fields = Fields("MoveToPose", default_fields, required_fields, optional_fields, 0, "new MoveToPose({})")
    fields.write_fields(f, '    ')
    fields.write_constructor(f, '    ', True)
    fields.write_methods(f, '    ')
    for i in range(1, 2**num_fields - 1):
        fields = Fields("MoveToPose", default_fields, required_fields, optional_fields, i, "new MoveToPose({})")

        name = fields.builder_state_name()

        f.write('\n    /** generated */\n')
        f.write('    public static class {} {{\n'.format(name))
        fields.write_fields(f, '        ')
        fields.write_constructor(f, '        ', False)
        fields.write_methods(f, '        ')
        f.write('    }\n')
    f.write('}\n')

first = Fields("MoveToPose", default_fields, required_fields, optional_fields, 0, "new MoveToPose({})")
second = first.next(2)
third = second.next(1)

print(first.builder_state_name())
print(second.builder_state_name())
print(third.builder_state_name())

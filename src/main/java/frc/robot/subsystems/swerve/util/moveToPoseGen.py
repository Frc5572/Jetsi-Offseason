
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

required_fields = [
    ["EventLoop", "eventLoop", []],
    ["Swerve", "swerve", []],
    ["Consumer<ChassisSpeeds>", "robotRelativeConsumer", []],
    ["Supplier<Pose2d>", "pose2dSupplier", ["Pose2d", "pose2d", "() -> pose2d"]],
]

optional_fields = [
    ["DoubleSupplier", "maxSpeedSupplier", "() -> Constants.Swerve.autoMaxSpeed", ["double", "maxSpeed", "() -> maxSpeed"]],
    ["boolean", "flipForRed", "true"],
    ["double", "translationTolerance", "0.5"],
    ["double", "rotationTolerance", "Units.degreesToRadians(5)"]
]

class Fields:
    def __init__(self, name, required_fields, optional_fields, permutation, constructor):
        self.name = name
        self.permutation = permutation
        self.required_set = [((permutation >> j) & 0x1) != 0 for j in range(0, len(required_fields))]
        self.optional_set = [((permutation >> (j + len(required_fields))) & 0x1) != 0 for j in range(0, len(optional_fields))]
        self.required_fields = required_fields
        self.optional_fields = optional_fields
        self.constructor = constructor
    
    def next(self, index):
        return Fields(self.name, self.required_fields, self.optional_fields, self.permutation | (1 << index), self.constructor)
    
    def is_full(self):
        return all(self.required_set) and all(self.optional_set)
    
    def could_finish(self):
        return all(self.required_set)

    def write_constructor(self, f, indent):
        f.write('\n{}private {}('.format(indent, self.builder_state_name()))
        has_args = False
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
        for i in range(0, len(self.required_fields)):
            if self.required_set[i]:
                f.write('{}    this.{} = {};\n'.format(indent, self.required_fields[i][1], self.required_fields[i][1]))
        for i in range(0, len(self.optional_fields)):
            if self.optional_set[i]:
                f.write('{}    this.{} = {};\n'.format(indent, self.optional_fields[i][1], self.optional_fields[i][1]))
        f.write('{}}}\n'.format(indent))
        pass

    def write_fields(self, f, indent):
        for i in range(0, len(self.required_fields)):
            if self.required_set[i]:
                f.write('{}private final {} {};\n'.format(indent, self.required_fields[i][0], self.required_fields[i][1]))
        for i in range(0, len(self.optional_fields)):
            if self.optional_set[i]:
                f.write('{}private final {} {};\n'.format(indent, self.optional_fields[i][0], self.optional_fields[i][1]))
        pass
    
    def write_methods(self, f, indent, is_static):
        if self.could_finish():
            f.write('\n{}public {}{} finish() {{\n'.format(indent, "static " if is_static else "", self.name))
            args = []
            for i in range(0, len(self.required_fields)):
                args.append(self.required_fields[i][1])
            for i in range(0, len(self.optional_fields)):
                if self.optional_set[i]:
                    args.append(self.optional_fields[i][1])
                else:
                    args.append(self.optional_fields[i][2])
            f.write('{}    return {};\n'.format(indent, self.constructor.format(', '.join(args))))
            f.write('{}}}\n'.format(indent))
        
        for i in range(0, len(self.required_fields)):
            if (self.permutation >> i) & 0x1 != 0:
                continue    

            ret = self.next(i)
            if ret.is_full():
                f.write('\n{}public {}{} {}({} {}) {{\n'.format(indent, "static " if is_static else "", self.name, self.required_fields[i][1], self.required_fields[i][0], self.required_fields[i][1]))
                args = []
                for i in range(0, len(self.required_fields)):
                    args.append(self.required_fields[i][1])
                for i in range(0, len(self.optional_fields)):
                    args.append(self.optional_fields[i][1])
                f.write('{}    return {};\n'.format(indent, self.constructor.format(', '.join(args))))
                f.write('{}}}\n'.format(indent))
            else:
                f.write('\n{}public {}{} {}({} {}) {{\n'.format(indent, "static " if is_static else "", ret.builder_state_name(), self.required_fields[i][1], self.required_fields[i][0], self.required_fields[i][1]))
                f.write('{}    return new {}('.format(indent, ret.builder_state_name()))
                written_arg = False
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
    
    f.write('\npublic class MoveToPoseBuilder {\n')
    fields = Fields("MoveToPose", required_fields, optional_fields, 0, "new MoveToPose({})")
    fields.write_methods(f, '    ', True)
    for i in range(1, 2**num_fields - 1):
        fields = Fields("MoveToPose", required_fields, optional_fields, i, "new MoveToPose({})")

        name = fields.builder_state_name()
        
        f.write('    public static class {} {{\n'.format(name))
        fields.write_fields(f, '        ')
        fields.write_constructor(f, '        ')
        fields.write_methods(f, '        ', False)
        f.write('    }\n')
    f.write('}\n')

first = Fields("MoveToPose", required_fields, optional_fields, 0, "new MoveToPose({})")
second = first.next(2)
third = second.next(1)

print(first.builder_state_name())
print(second.builder_state_name())
print(third.builder_state_name())

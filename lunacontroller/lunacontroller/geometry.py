import math
from typing import Union

class Translation2d:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
    def __add__(self, other: 'Translation2d') -> 'Translation2d':
        return Translation2d(self.x + other.x, self.y + other.y)
    def __sub__(self, other: 'Translation2d') -> 'Translation2d':
        return Translation2d(self.x - other.x, self.y - other.y)
    def __mul__(self, scalar: float) -> 'Translation2d':
        return Translation2d(self.x * scalar, self.y * scalar)
    def add(self, other: 'Translation2d') -> 'Translation2d':
        return self + other
    def sub(self, other: 'Translation2d') -> 'Translation2d':
        return self - other
    def multiply(self, scalar: float) -> 'Translation2d':
        return self * scalar
    def distance(self, other: 'Translation2d') -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5
    def magnitude(self) -> float:
        return (self.x ** 2 + self.y ** 2) ** 0.5
    def normalize(self) -> 'Translation2d':
        mag = self.magnitude()
        if mag == 0:
            return Translation2d(0, 0)
        return Translation2d(self.x / mag, self.y / mag)
    def rotate(self, angle: "Rotation2d") -> 'Translation2d':
        cos_a = angle.cos()
        sin_a = angle.sin()
        return Translation2d(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a
        )

class Rotation2d:
    def __init__(self, radians: float):
        self.radians = radians
    def cos(self) -> float:
        return math.cos(self.radians)
    def sin(self) -> float:
        return math.sin(self.radians)
    def add(self, other: 'Rotation2d') -> 'Rotation2d':
        return Rotation2d(self.radians + other.radians)
    def sub(self, other: 'Rotation2d') -> 'Rotation2d':
        return Rotation2d(self.radians - other.radians)
    def multiply(self, scalar: float) -> 'Rotation2d':
        return Rotation2d(self.radians * scalar)
    def __add__(self, other: 'Rotation2d') -> 'Rotation2d':
        return self.add(other)
    def __sub__(self, other: 'Rotation2d') -> 'Rotation2d':
        return self.sub(other)
    def __mul__(self, scalar: float) -> 'Rotation2d':
        return self.multiply(scalar)
    @staticmethod
    def from_degrees(degrees: float) -> 'Rotation2d':
        return Rotation2d(math.radians(degrees))
    def get_degrees(self) -> float:
        return math.degrees(self.radians)

class Pose2d:
    def __init__(self, translation_or_x: Union[Translation2d, float], rotation_or_y: Union[Rotation2d, float], maybe_radians: float = None):
        if isinstance(translation_or_x, Translation2d) and isinstance(rotation_or_y, Rotation2d):
            self.translation = translation_or_x
            self.rotation = rotation_or_y
        else:
            x = float(translation_or_x)
            y = float(rotation_or_y)
            radians = float(maybe_radians)
            self.translation = Translation2d(x, y)
            self.rotation = Rotation2d(radians)
    def transform_by(self, other: 'Pose2d') -> 'Pose2d':
        new_translation = self.translation.add(other.translation.rotate(self.rotation))
        new_rotation = self.rotation.add(other.rotation)
        return Pose2d(new_translation, new_rotation)
    def distance(self, other: 'Pose2d') -> float:
        return self.translation.distance(other.translation)
    def __add__(self, other: 'Pose2d') -> 'Pose2d':
        return self.transform_by(other)
    def __sub__(self, other: 'Pose2d') -> 'Pose2d':
        inv_other_translation = other.translation.multiply(-1).rotate(Rotation2d(-other.rotation.radians))
        inv_other_rotation = Rotation2d(-other.rotation.radians)
        return self.transform_by(Pose2d(inv_other_translation, inv_other_rotation))
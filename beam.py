import abc


class _Crossection:
    """Assume Center of Mass (COM) at x=0, y=0.
    
    Assum moment of inertia around y=0.
    """
    @property
    def com(self) -> tuple[float, float]:
        """Center of mass."""
        return 0, 0

    @abc.abstractproperty
    def area(self) -> float:
        raise NotImplementedError

    @abc.abstractproperty
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Bounding box.
        
        ((xmin, ymin), (xmax, ymax))
        """
        raise NotImplementedError

    @abc.abstractproperty
    def mom(self) -> float:
        """Moment of Inertia around the x-axis (y=0)."""
        raise NotImplementedError

    def shift(self, offset: tuple[float, float]):
        return _ShiftedCrosssection(self, offset)

    def scale(self, scale: float | tuple[float, float]):
        if isinstance(scale, float):
            scale = (scale, scale)
        return _ScaledCrossection(self, scale)

    def __mul__(self, weight: float):
        assert isinstance(weight, (float, int)), f"{weight=}"
        return _WeightedCrossection(self, weight)

    def __rmul__(self, weight: float):
        return self * weight

    def __neg__(self):
        return (-1) * self

    def __add__(self, other):
        return _SumCrossection(self, other)
    
    def __sub__(self, other):
        return self + (-other)

    def center(self):
        x, y = self.com
        return self.shift((-x, -y))


class _ShiftedCrosssection(_Crossection):
    def __init__(self, crossection: _Crossection, offset: tuple[float, float]):
        self.crossection = crossection
        self.offset = offset

    @property
    def area(self) -> float:
        return self.crossection.area

    @property
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        ((xmin, ymin), (xmax, ymax)) = self.crossection.bb
        xshift, yshift = self.offset
        return ((xmin + xshift, ymin + yshift), (xmax + xshift, ymax + yshift)) 

    @property
    def mom(self) -> float:
        return self.crossection.mom + self.crossection.area * self.offset[1] * self.offset[1]

    @property
    def com(self) -> tuple[float, float]:
        return self.offset[0] + self.crossection.com[0], self.offset[1] + self.crossection.com[1]


class _ScaledCrossection(_Crossection):
    def __init__(self, crossection: _Crossection, scale: tuple[float, float]):
        self.crossection = crossection
        self.scale = scale

    @property
    def area(self) -> float:
        return self.crossection.area * self.scale[0] * self.scale[1]

    @property
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        ((xmin, ymin), (xmax, ymax)) = self.crossection.bb
        xscale, yscale = self.scale
        return ((xmin * xscale, ymin * yscale), (xmax * xscale, ymax * yscale)) 

    @property
    def mom(self) -> float:
        return self.crossection.mom * self.scale[0] * (self.scale[1] ** 3)

    @property
    def com(self) -> tuple[float, float]:
        return self.crossection.com


class _WeightedCrossection(_Crossection):
    def __init__(self, crossection: _Crossection, weight: float):
        self.crossection = crossection
        self.weight = weight

    @property
    def area(self) -> float:
        return self.crossection.area * self.weight

    @property
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        return self.crossection.bb

    @property
    def mom(self) -> float:
        return self.crossection.mom * self.weight

    @property
    def com(self) -> tuple[float, float]:
        return self.crossection.com


class _SumCrossection(_Crossection):
    def __init__(self, first, second) -> None:
        self.first = first
        self.second = second

    @property
    def area(self) -> float:
        return self.first.area + self.second.area

    @property
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        ((xmin1, ymin1), (xmax1, ymax1)) = self.first.bb
        ((xmin2, ymin2), (xmax2, ymax2)) = self.second.bb

        return ((min(xmin1, xmin2), min(ymin1, ymin2)), (max(xmax1, xmax2), max(ymax1, ymax2))) 

    @property
    def mom(self) -> float:
        return self.first.mom + self.second.mom

    @property
    def com(self) -> tuple[float, float]:
        x1, y1 = self.first.com
        m1 = self.first.area
        x2, y2 = self.second.com
        m2 = self.second.area
        return (x1 * m1 + x2 * m2) / (m1 + m2), (y1 * m1 + y2 * m2) / (m1 + m2)


class _CompositeSection(_Crossection):
    @abc.abstractproperty
    def _composite(self) -> _Crossection:
        raise NotImplementedError

    @property
    def area(self) -> float:
        return self._composite.area

    @property
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        return self._composite.bb

    @property
    def mom(self) -> float:
        return self._composite.mom

    @property
    def com(self) -> tuple[float, float]:
        return self._composite.com


class _UnitSquare(_Crossection):
    @property
    def area(self) -> float:
        return 1

    @property
    def bb(self) -> tuple[tuple[float, float], tuple[float, float]]:
        return ((-0.5, -0.5), (0.5, 0.5))

    @property
    def mom(self) -> float:
        return 1. / 12

    @property
    def com(self) -> tuple[float, float]:
        return 0, 0


class Rectangle(_CompositeSection):
    def __init__(self, x_side, y_side) -> None:
        self.x_side, self.y_side= x_side, y_side
    
    @property
    def _composite(self):
        return _UnitSquare().scale((self.x_side, self.y_side))


class AngleSection(_CompositeSection):
    def __init__(self, side, wall) -> None:
        self.side, self.wall = side, wall

    @property
    def _composite(self):
        side, wall = self.side, self.wall
        r = Rectangle(side, side) - Rectangle(side - wall, side - wall).shift((wall/2, wall/2))
        return r.center()

class IBeam(_CompositeSection):
    def __init__(self, width, height, flange, web) -> None:
        self.width, self.height, self.flange, self.web = width, height, flange, web

    @property
    def _composite(self):
        width, height, flange, web = self.width, self.height, self.flange, self.web
        mid = Rectangle(web, height - 2 * flange)
        top = Rectangle(width, flange).shift((0, (height - flange) / 2))
        bottom = Rectangle(width, flange).shift((0, -(height - flange) / 2))
        return top + mid + bottom


#### Calculate


# https://mechanicalc.com/reference/beam-analysis
def simply_supported_point_load(section: _Crossection, L: float, E: float, F: float):
    I = section.mom
    c = max(abs(section.bb[0][1]), abs(section.bb[1][1]))  # max y distance
    M_max = F * L / 4
    max_deflection = - F *  L**3 / (48 * E * I)  # at midpoint
    max_stress = M_max * c / I
    return {"c": c, "max_deflection": max_deflection, "max_stress": max_stress}


# steel S235 - 207GPa
GPa = 1e9
m = 1.
mm = 0.001
cm = 0.01
E = 207 * GPa  #
Y = 345 * GPa  # yield strength
rho = 7850
section = IBeam(width=100*mm, height=100*mm, flange=3*mm, web=3*mm)
# section = AngleSection(side=40*mm, wall=5*mm)

g = 9.81
L = 4*m

if __name__ == "__main__":
    print(f"{section.com=}")
    print(f"{section.mom ** 0.25=}")
    print(f"{section.bb=}")
    print(f"{section.area=}")
    print(f"{section.area * L * rho=} kg")
    result = simply_supported_point_load(section, L=L, E=E, F=1000 * g)
    print(result)
    print(f"% of Y: {result['max_stress'] / Y}")

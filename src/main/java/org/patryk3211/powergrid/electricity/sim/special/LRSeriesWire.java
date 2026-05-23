/*
 * Copyright 2025 patryk3211
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.patryk3211.powergrid.electricity.sim.special;

import org.patryk3211.powergrid.electricity.sim.AbstractElectricWire;
import org.patryk3211.powergrid.electricity.sim.node.IElectricNode;
import org.patryk3211.powergrid.electricity.sim.node.ITimeAwareWire;
import org.patryk3211.powergrid.electricity.sim.solver.IOuterHook;
import org.patryk3211.powergrid.electricity.sim.solver.IResidualAdder;
import org.patryk3211.powergrid.electricity.sim.solver.IStaticResidual;

public class LRSeriesWire extends AbstractElectricWire implements IStaticResidual, IOuterHook, ITimeAwareWire {
    private double inductance;
    private double resistance;

    private double Ieq;
    private double I;
    private double Vprev;

    public LRSeriesWire(double L, double R, IElectricNode node1, IElectricNode node2) {
        super(node1, node2);
        this.inductance = L;
        this.resistance = R;
    }

    private boolean isStiff() {
        if (inductance == 0 || resistance == 0) return false;
        return getDeltaTime() * resistance / inductance >= 0.5;
    }

    @Override
    public double conductance() {
        double G_ind = isStiff()
            ? getDeltaTime() / inductance
            : getDeltaTime() * 2.0 / (3.0 * inductance);
        return 1.0 / (resistance + 1.0 / G_ind);
    }

    @Override
    public double current() {
        if(network == null)
            return I;
        if(network.isLeaf(node1) || network.isLeaf(node2))
            return 0;
        return super.current() + Ieq;
    }

    public void setCurrent(float current) {
        valueChange(current, I);
        if(Float.isFinite(current)) {
            Vprev = 0;
            I = current;
        }
    }

    @Override
    public void postUpperSolve() {
        if(isConverged()) {
            double Inew = current();
            if(isStiff()) {
                Vprev = 0;
            } else {
                Vprev = inductance * (Inew - I) / (2.0 * getDeltaTime());
            }
            I = Inew * 0.99999;
        }
    }

    @Override
    public double getLocalTau() {
        return (inductance == 0 || resistance == 0) ? Double.MAX_VALUE : inductance / resistance;
    }

    @Override
    public void addStaticResidual(IResidualAdder residual) {
        if(inductance == 0) {
            Ieq = 0;
            return;
        }
        var G_I = isStiff()
            ? getDeltaTime() / inductance
            : getDeltaTime() * 2.0 / (3.0 * inductance);

        double residualScale = 1 - G_I / (1 / resistance + G_I);
        Ieq = (Vprev * G_I + I) * residualScale;
        if(node1 != null)
            residual.add(node1.getIndex(), -Ieq);
        if(node2 != null)
            residual.add(node2.getIndex(),  Ieq);
    }

    public void setLR(double L, double R) {
        var oldConductance = conductance();
        this.inductance = L;
        this.resistance = R;

        if(network != null) {
            network.updateConductance(this, conductance() - oldConductance);
        }
    }

    public void setInductance(double L) {
        setLR(L, resistance);
    }

    public void setResistance(double R) {
        setLR(inductance, R);
    }

    @Override
    public String toString() {
        return String.format("LRWire(L=%g R=%g)", inductance, resistance);
    }
}

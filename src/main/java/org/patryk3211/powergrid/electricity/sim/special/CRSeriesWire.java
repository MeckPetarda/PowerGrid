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

public class CRSeriesWire extends AbstractElectricWire implements IStaticResidual, IOuterHook, ITimeAwareWire {
    private double capacitance;
    private double resistance;

    private double Ieq;
    private double Iprev;
    private double V;

    public CRSeriesWire(double C, double R, IElectricNode node1, IElectricNode node2) {
        super(node1, node2);
        this.capacitance = C;
        this.resistance = R;
    }

    @Override
    public boolean isSource() {
        return true;
    }

    public void setVoltage(float voltage) {
        valueChange(voltage, V);
        if(Float.isFinite(voltage)) {
            Iprev = 0;
            V = voltage;
        }
    }

    @Override
    public double potentialDifference() {
        if(network == null)
            return V;
        return super.potentialDifference();
    }

    public double capacitorVoltage() {
        return potentialDifference() - current() * resistance;
    }

    private boolean isStiff() {
        if (capacitance == 0 || resistance == 0) return false;
        return getDeltaTime() / (resistance * capacitance) >= 0.5;
    }

    @Override
    public double conductance() {
        double G_cap = isStiff()
            ? capacitance / getDeltaTime()
            : 1.5 * capacitance / getDeltaTime();
        return 1.0 / (resistance + 1.0 / G_cap);
    }

    @Override
    public double current() {
        return super.current() + Ieq;
    }

    @Override
    public void postUpperSolve() {
        if(isConverged()) {
            double Vcap = capacitorVoltage();
            if(isStiff()) {
                Iprev = 0;
            } else {
                Iprev = capacitance * (Vcap - V) / (2.0 * getDeltaTime());
            }
            V = Vcap * 0.99999;
        }
    }

    @Override
    public double getLocalTau() {
        return (capacitance == 0 || resistance == 0) ? Double.MAX_VALUE : resistance * capacitance;
    }

    @Override
    public void addStaticResidual(IResidualAdder residual) {
        if(capacitance == 0) {
            Ieq = 0;
            return;
        }
        var G_C = isStiff()
            ? capacitance / getDeltaTime()
            : 1.5 * capacitance / getDeltaTime();

        double residualScale = 1 - G_C / (1 / resistance + G_C);
        Ieq = (-G_C * V - Iprev) * residualScale;
        if(node1 != null)
            residual.add(node1.getIndex(), -Ieq);
        if(node2 != null)
            residual.add(node2.getIndex(),  Ieq);
    }

    public void setCR(double C, double R) {
        var oldConductance = conductance();
        this.capacitance = C;
        this.resistance = R;

        if(network != null) {
            network.updateConductance(this, conductance() - oldConductance);
        }
    }

    public void setCapacitance(double C) {
        setCR(C, resistance);
    }

    public void setResistance(double R) {
        setCR(capacitance, R);
    }

    @Override
    public String toString() {
        return String.format("CRWire(C=%g R=%g)", capacitance, resistance);
    }
}

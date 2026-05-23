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
package org.patryk3211.powergrid.kinetics.plotter;

import com.google.common.collect.ImmutableList;
import com.simibubi.create.api.behaviour.display.DisplaySource;
import com.simibubi.create.content.redstone.displayLink.DisplayLinkContext;
import com.simibubi.create.content.redstone.displayLink.target.DisplayTargetStats;
import com.simibubi.create.foundation.gui.ModularGuiLineBuilder;
import net.fabricmc.api.EnvType;
import net.fabricmc.api.Environment;
import net.minecraft.network.chat.Component;
import net.minecraft.network.chat.MutableComponent;
import org.patryk3211.powergrid.utility.Lang;

import java.util.List;

public class PlotterDisplaySource extends DisplaySource {

    @Override
    public List<MutableComponent> provideText(DisplayLinkContext context, DisplayTargetStats stats) {
        if (!(context.getSourceBlockEntity() instanceof PlotterBlockEntity plotter))
            return EMPTY;
        if (context.sourceConfig().getInt("Mode") == 1)
            return statsLines(plotter);
        return ImmutableList.of(plotter.displayBehaviour.format(currentVoltage(plotter)).component());
    }

    private float currentVoltage(PlotterBlockEntity plotter) {
        if (plotter.sampleBuffer.length == 0) return 0;
        int prev = (plotter.head - 1 + plotter.sampleBuffer.length) % plotter.sampleBuffer.length;
        return plotter.sampleBuffer[prev] * plotter.maxValue;
    }

    private List<MutableComponent> statsLines(PlotterBlockEntity plotter) {
        float[] buf = plotter.sampleBuffer;
        float scale = plotter.maxValue;
        if (buf.length == 0) return EMPTY;

        float max = Float.NEGATIVE_INFINITY, min = Float.POSITIVE_INFINITY, sum = 0;
        for (float v : buf) {
            float voltage = v * scale;
            if (voltage > max) max = voltage;
            if (voltage < min) min = voltage;
            sum += voltage;
        }
        float t = buf.length / 20f;

        return ImmutableList.of(
            labeled("now", currentVoltage(plotter), plotter),
            labeled("max", max, plotter),
            labeled("min", min, plotter),
            labeled("avg", sum / buf.length, plotter),
            Lang.text("t ").add(Lang.numberConstant(t)).add(Component.literal("s")).component()
        );
    }

    private MutableComponent labeled(String label, float voltage, PlotterBlockEntity plotter) {
        return Lang.text(label + " ").add(plotter.displayBehaviour.format(voltage)).component();
    }

    @Override
    public int getPassiveRefreshTicks() {
        return 20;
    }

    @Override
    protected String getTranslationKey() {
        return "plotter";
    }

    @Override
    @Environment(EnvType.CLIENT)
    public void initConfigurationWidgets(DisplayLinkContext context, ModularGuiLineBuilder builder, boolean isFirstLine) {
        if (isFirstLine) return;
        builder.addSelectionScrollInput(0, 120, (si, l) -> si
            .forOptions(Lang.translatedOptions("display_source.plotter", "value", "stats"))
            .titled(Lang.translateDirect("display_source.display_information")),
            "Mode");
    }
}

package org.firstinspires.ftc.teamcode.Tests;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class SampleTest {
    @Test
    void addSmth(){
        Sample sample = new Sample();
        assertEquals(sample.add(2,4), 6);
    }
}
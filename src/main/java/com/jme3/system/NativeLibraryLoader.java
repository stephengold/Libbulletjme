/*
 * Copyright (c) 2009-2019 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.system;

import java.io.File;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Utility class to load native libraries.
 *
 * @author Kirill Vainer
 */
public final class NativeLibraryLoader {

    final public static Logger logger
            = Logger.getLogger(NativeLibraryLoader.class.getName());

    private NativeLibraryLoader() {
    }

    /**
     * Load a Libbulletjme native library.
     *
     * @param dist true&rarr;distributed files, false&rarr;as-built files
     * @param directory (not null, readable, unaffected)
     * @param buildType "Debug" or "Release"
     * @param flavor "Sp" or "Dp"
     */
    public static void loadLibbulletjme(boolean dist, File directory,
            String buildType, String flavor) {
        assert buildType.equals("Debug") || buildType.equals("Release");
        assert flavor.equals("Sp") || flavor.equals("Dp");

        Platform platform = JmeSystem.getPlatform();

        String name;
        switch (platform) {
            case Windows32:
            case Windows64:
                name = "bulletjme.dll";
                break;
            case Linux_ARM32:
            case Linux_ARM64:
            case Linux32:
            case Linux64:
                name = "libbulletjme.so";
                break;
            case MacOSX32:
            case MacOSX64:
                name = "libbulletjme.dylib";
                break;
            default:
                throw new RuntimeException("platform = " + platform);
        }

        File file;
        if (dist) {
            name = platform + buildType + flavor + "_" + name;
            file = directory;

        } else {
            String subdirectory = firstToLower(platform.toString());
            file = new File(directory, subdirectory);

            String bt = buildType.toLowerCase();
            file = new File(file, bt);

            String f = flavor.toLowerCase();
            file = new File(file, f);
        }

        file = new File(file, name);
        String absoluteFilename = file.getAbsolutePath();
        if (!file.exists()) {
            logger.log(Level.SEVERE, "{0} does not exist", absoluteFilename);
        } else if (!file.canRead()) {
            logger.log(Level.SEVERE, "{0} is not readable", absoluteFilename);
        } else {
            System.load(absoluteFilename);
        }
    }

    /**
     * Convert the first character of the specified String to lower case.
     *
     * @param input the input string (not null)
     * @return the converted String (not null)
     */
    private static String firstToLower(String input) {
        String result = input;
        if (!input.isEmpty()) {
            String first = input.substring(0, 1);
            first = first.toLowerCase(Locale.ROOT);
            String rest = input.substring(1);
            result = first + rest;
        }

        return result;
    }
}

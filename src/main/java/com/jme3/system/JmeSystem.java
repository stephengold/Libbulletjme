/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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

/**
 * Utility class to access platform-dependent features.
 */
public class JmeSystem {
    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private JmeSystem() {
    }

    private static boolean is64Bit(String arch) {
        if (arch.equals("x86")) {
            return false;
        } else if (arch.equals("amd64")) {
            return true;
        } else if (arch.equals("x86_64")) {
            return true;
        } else if (arch.equals("ppc") || arch.equals("PowerPC")) {
            return false;
        } else if (arch.equals("ppc64")) {
            return true;
        } else if (arch.equals("i386") || arch.equals("i686")) {
            return false;
        } else if (arch.equals("universal")) {
            return false;
        } else if (arch.equals("aarch32")) {
            return false;
        } else if (arch.equals("aarch64")) {
            return true;
        } else if (arch.equals("armv7") || arch.equals("armv7l")) {
            return false;
        } else if (arch.equals("arm")) {
            return false;
        } else if (arch.equals("loong64") || arch.equals("loongarch64")) {
            return true;
        } else {
            throw new UnsupportedOperationException("Unsupported architecture: " + arch);
        }
    }

    /**
     * Determine which Platform (operating system and architecture) the
     * application is running on.
     *
     * @return an enum value (not null)
     */
    public static Platform getPlatform() {
        String os = System.getProperty("os.name").toLowerCase();
        String arch = System.getProperty("os.arch").toLowerCase();
        boolean is64 = is64Bit(arch);
        if (os.contains("windows")) {
            if (arch.startsWith("arm") || arch.startsWith("aarch")) {
                return is64 ? Platform.Windows_ARM64 : Platform.Windows_ARM32;
            } else {
                return is64 ? Platform.Windows64 : Platform.Windows32;
            }
        } else if (os.contains("linux") || os.contains("freebsd")
                || os.contains("sunos") || os.contains("unix")) {
            if (arch.startsWith("arm") || arch.startsWith("aarch")) {
                return is64 ? Platform.Linux_ARM64 : Platform.Linux_ARM32;
            } else if (arch.startsWith("loong")) {
                return Platform.Linux_LoongArch64; // currently 32-bit version not supported
            } else {
                return is64 ? Platform.Linux64 : Platform.Linux32;
            }
        } else if (os.contains("mac os x") || os.contains("darwin")) {
            if (arch.startsWith("ppc")) {
                return is64 ? Platform.MacOSX_PPC64 : Platform.MacOSX_PPC32;
            } else if (arch.startsWith("aarch")) {
                return Platform.MacOSX_ARM64; // no 32-bit version
            } else {
                return is64 ? Platform.MacOSX64 : Platform.MacOSX32;
            }
        } else {
            throw new UnsupportedOperationException("The specified platform: " + os + " is not supported.");
        }
    }
}

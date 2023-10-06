/*
 Copyright (c) 2013-2023, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.debug;

import java.io.PrintStream;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Dump portions of a Libbulletjme object for debugging.
 * <p>
 * The level of detail can be configured dynamically.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Dumper implements Cloneable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(Dumper.class.getName());
    // *************************************************************************
    // fields

    /**
     * describer for Libbulletjme objects
     */
    private Describer describer;
    /**
     * stream to use for output: set by constructor
     */
    final protected PrintStream stream;
    /**
     * indentation increment for each level of a dump
     */
    private String indentIncrement = "  ";
    // *************************************************************************
    // constructors

    /**
     * Instantiate a dumper that will use System.out for output.
     */
    public Dumper() {
        this.describer = new Describer();
        this.stream = System.out;
    }

    /**
     * Instantiate a dumper that will use the specified output stream.
     *
     * @param printStream the output stream (not null, alias created)
     */
    public Dumper(PrintStream printStream) {
        Validate.nonNull(printStream, "print stream");

        this.describer = new Describer();
        this.stream = printStream;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the Describer used by this dumper.
     *
     * @return the pre-existing instance (not null)
     */
    public Describer getDescriber() {
        assert describer != null;
        return describer;
    }

    /**
     * Read the indent increment.
     *
     * @return (not null, may be empty)
     */
    public String indentIncrement() {
        assert indentIncrement != null;
        return indentIncrement;
    }

    /**
     * Alter which describer to use.
     *
     * @param newDescriber (not null, alias created)
     * @return this instance for chaining
     */
    public Dumper setDescriber(Describer newDescriber) {
        Validate.nonNull(newDescriber, "new describer");
        this.describer = newDescriber;
        return this;
    }

    /**
     * Configure the indent increment.
     *
     * @param newValue (not null, default=" ")
     * @return this instance for chaining
     */
    public Dumper setIndentIncrement(String newValue) {
        Validate.nonNull(newValue, "increment");
        this.indentIncrement = newValue;
        return this;
    }
    // *************************************************************************
    // new protected methods

    /**
     * If the specified description is non-empty, print it to the stream,
     * prefixed by a blank.
     *
     * @param description (not null)
     */
    protected void addDescription(String description) {
        Validate.nonNull(description, "description");
        if (!description.isEmpty()) {
            stream.print(' ');
            stream.print(description);
        }
    }

    /**
     * Print a newline, followed by the specified indentation.
     *
     * @param indent (not null)
     */
    protected void addLine(String indent) {
        Validate.nonNull(indent, "indent");

        stream.println();
        stream.print(indent);
    }
    // *************************************************************************
    // Cloneable methods

    /**
     * Create a deep copy of this Dumper.
     *
     * @return a new instance, equivalent to this one, with its own Describer
     * @throws CloneNotSupportedException if the superclass isn't cloneable
     */
    @Override
    public Dumper clone() throws CloneNotSupportedException {
        Dumper clone = (Dumper) super.clone();
        this.describer = describer.clone();
        // stream not cloned

        return clone;
    }
}

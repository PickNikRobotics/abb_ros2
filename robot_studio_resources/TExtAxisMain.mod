MODULE TExtAxisMain
!======================================================================================================
! Copyright (c) 2018, ABB Schweiz AG
! All rights reserved.
!
! Redistribution and use in source and binary forms, with
! or without modification, are permitted provided that
! the following conditions are met:
!
!    * Redistributions of source code must retain the
!      above copyright notice, this list of conditions
!      and the following disclaimer.
!    * Redistributions in binary form must reproduce the
!      above copyright notice, this list of conditions
!      and the following disclaimer in the documentation
!      and/or other materials provided with the
!      distribution.
!    * Neither the name of ABB nor the names of its
!      contributors may be used to endorse or promote
!      products derived from this software without
!      specific prior wrSitten permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
! DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
! CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
! OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
! THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
!======================================================================================================
    
    !***********************************************************
    ! Program data
    !***********************************************************
    ! Home position.
    LOCAL CONST jointtarget home := [[0, 0, 0, 0, 30, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
    
    ! Identifier for the EGM correction.
    LOCAL VAR egmident egm_id;
    
    ! Limits for convergance.
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];
            
    !***********************************************************
    !
    ! Procedure main
    !
    !   This RAPID code exemplify how to run EGM joint position
    !   motions.
    !
    !   Note: Update the UCDevice "EXTAX" with correct
    !         values for the remote address and port
    !         (i.e. to the EGM server).
    !
    !         Update via RobotStudio:
    !         Controller tab -> Configuration ->
    !         Communication -> Transmission Protocol
    !
    !***********************************************************
    PROC main()
        WHILE TRUE DO
            
            ! Register an EGM id.
            EGMGetId egm_id;
            
            ! Setup the EGM communication.
            EGMSetupUC MU_250__01, egm_id, "default", "EXTAX", \Joint;
            
            ! Prepare for an EGM communication session.
            EGMActJoint egm_id;
            
            ! Start the EGM communication session.
            EGMRunJoint egm_id, EGM_STOP_HOLD;
            
            ! Release the EGM id.
            EGMReset egm_id;
            
            WaitTime 5;
        ENDWHILE
        
    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE